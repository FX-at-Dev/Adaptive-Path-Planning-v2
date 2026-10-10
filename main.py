import argparse
import ctypes
import json
import os
import shutil
import socket
import struct
import subprocess
import sys
import threading
import time

SPEED_LIMIT = None
PRINT_EVERY = 1.0


def drive(t, state, speed, accel, steer, reason):
    if SPEED_LIMIT is not None and speed > SPEED_LIMIT and accel > -1.5:
        accel = -1.5
    return accel, steer


def read_exact(sock, n):
    buf = bytearray()
    while len(buf) < n:
        chunk = sock.recv(n - len(buf))
        if not chunk:
            raise ConnectionError("closed")
        buf.extend(chunk)
    return bytes(buf)


def read_frame(sock):
    nj, nb = struct.unpack("<II", read_exact(sock, 8))
    return read_exact(sock, nj), read_exact(sock, nb)


def write_frame(sock, js, blob):
    sock.sendall(struct.pack("<II", len(js), len(blob)) + js + blob)


def find_octave():
    exe = shutil.which("octave-cli")
    if exe:
        return exe
    root = os.path.join(os.environ.get("LOCALAPPDATA", ""), "Programs", "GNU Octave")
    if os.path.isdir(root):
        for ver in sorted(os.listdir(root), reverse=True):
            cand = os.path.join(root, ver, "mingw64", "bin", "octave-cli.exe")
            if os.path.isfile(cand):
                return cand
    return None


def full_speed(proc):
    if os.name != "nt":
        return
    try:
        class PPT(ctypes.Structure):
            _fields_ = [("Version", ctypes.c_uint), ("ControlMask", ctypes.c_uint), ("StateMask", ctypes.c_uint)]
        k32 = ctypes.windll.kernel32
        h = k32.OpenProcess(0x0200 | 0x0400, False, proc.pid)
        p = PPT(1, 1, 0)
        k32.SetProcessInformation(h, 4, ctypes.byref(p), ctypes.sizeof(p))
        k32.SetPriorityClass(h, 0x8000)
        k32.CloseHandle(h)
    except Exception:
        pass


def tie_to_this_script(proc):
    if os.name != "nt":
        return None
    try:
        k32 = ctypes.windll.kernel32
        job = k32.CreateJobObjectW(None, None)

        class LIMIT(ctypes.Structure):
            _fields_ = [("PerProcessUserTimeLimit", ctypes.c_int64), ("PerJobUserTimeLimit", ctypes.c_int64),
                        ("LimitFlags", ctypes.c_uint32), ("MinimumWorkingSetSize", ctypes.c_size_t),
                        ("MaximumWorkingSetSize", ctypes.c_size_t), ("ActiveProcessLimit", ctypes.c_uint32),
                        ("Affinity", ctypes.c_size_t), ("PriorityClass", ctypes.c_uint32),
                        ("SchedulingClass", ctypes.c_uint32)]

        class IO(ctypes.Structure):
            _fields_ = [(n, ctypes.c_uint64) for n in ("r", "w", "o", "rb", "wb", "ob")]

        class EXT(ctypes.Structure):
            _fields_ = [("Basic", LIMIT), ("Io", IO), ("ProcessMemoryLimit", ctypes.c_size_t),
                        ("JobMemoryLimit", ctypes.c_size_t), ("PeakProcessMemoryUsed", ctypes.c_size_t),
                        ("PeakJobMemoryUsed", ctypes.c_size_t)]

        info = EXT()
        info.Basic.LimitFlags = 0x2000
        k32.SetInformationJobObject(job, 9, ctypes.byref(info), ctypes.sizeof(info))
        h = k32.OpenProcess(0x1F0FFF, False, proc.pid)
        k32.AssignProcessToJobObject(job, h)
        k32.CloseHandle(h)
        return job
    except Exception:
        return None


def start_stack(octave, repo, port):
    root = repo.replace("\\", "/")
    ev = f"cd('{root}'); startup; ap_cosim_serve(struct('port', {port}, 'idle_exit_s', 900))"
    flags = 0x08000000 if os.name == "nt" else 0
    proc = subprocess.Popen([octave, "--no-gui", "--eval", ev], cwd=repo,
                            stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                            creationflags=flags, text=True, bufsize=1)
    full_speed(proc)
    start_stack.job = tie_to_this_script(proc)

    def echo():
        for line in proc.stdout:
            print("[stack] " + line.rstrip(), flush=True)
    threading.Thread(target=echo, daemon=True).start()

    def follow():
        code = proc.wait()
        print(f"main.py: the stack exited (code {code}); stopping", flush=True)
        os._exit(code)
    threading.Thread(target=follow, daemon=True).start()
    return proc


class Bridge:
    def __init__(self, unity_port, stack_port):
        self.unity_port = unity_port
        self.stack_port = stack_port
        self.last_print = -1e9
        self.speed = 0.0
        self.reason = ""

    def unity_to_stack(self, unity, stack):
        while True:
            js, blob = read_frame(unity)
            msg = json.loads(js)
            if msg.get("type") == "tick" and len(blob) >= 48:
                self.speed = struct.unpack_from("<6d", blob, 0)[3]
            write_frame(stack, js, blob)
            if msg.get("type") == "bye":
                return

    def stack_to_unity(self, stack, unity):
        while True:
            js, blob = read_frame(stack)
            msg = json.loads(js)
            kind = msg.get("type")
            if kind == "hello":
                msg["bridge"] = "main.py"
                print(f"main.py: driving {msg.get('scenario')} (world {msg.get('layout_seed')})", flush=True)
                js = json.dumps(msg).encode()
            elif kind == "act" and len(blob) >= 16:
                accel, steer = struct.unpack_from("<2d", blob, 0)
                series = msg.get("series", {}) or {}
                t = float(msg.get("t", 0.0))
                state = series.get("state", "")
                think = (msg.get("frame") or {}).get("think") or {}
                if think.get("reason"):
                    self.reason = think["reason"]
                new_accel, new_steer = drive(t, state, self.speed, accel, steer, self.reason)
                if (new_accel, new_steer) != (accel, steer):
                    blob = struct.pack("<2d", new_accel, new_steer) + blob[16:]
                if t - self.last_print >= PRINT_EVERY:
                    self.last_print = t
                    print(f"main.py: t {t:6.1f} s  {state:<7}  speed {self.speed * 3.6:5.1f} km/h  "
                          f"accel {new_accel:+5.2f}  steer {new_steer:+5.2f}  {self.reason}", flush=True)
                if msg.get("done"):
                    self.last_print = -1e9
            elif kind == "error":
                print(f"main.py: stack error: {msg.get('message')}", flush=True)
            write_frame(unity, js, blob)

    def serve(self):
        lst = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        if os.name == "nt":


            lst.setsockopt(socket.SOL_SOCKET, socket.SO_EXCLUSIVEADDRUSE, 1)
        else:
            lst.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        lst.bind(("127.0.0.1", self.stack_port))
        lst.listen(4)
        while True:
            unity = None
            while unity is None:
                try:
                    unity = socket.create_connection(("127.0.0.1", self.unity_port), timeout=2)
                    unity.settimeout(None)
                except OSError:
                    time.sleep(0.5)
            print(f"main.py: connected to Unity on port {self.unity_port}; waiting for the stack", flush=True)
            stack, ready = self.accept_stack(lst)
            print("main.py: the stack is connected -- every command now passes through drive()", flush=True)
            for s in (unity, stack):
                s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            try:
                write_frame(unity, *ready)
            except OSError:
                stack.close()
                continue
            a = threading.Thread(target=self._pump, args=(self.unity_to_stack, unity, stack), daemon=True)
            b = threading.Thread(target=self._pump, args=(self.stack_to_unity, stack, unity), daemon=True)
            a.start(); b.start()
            a.join(); b.join()
            print("main.py: Unity left; waiting for the next Play", flush=True)

    @staticmethod
    def accept_stack(lst):
        while True:
            s, _ = lst.accept()
            try:
                s.settimeout(5)
                ready = read_frame(s)
                s.setblocking(False)
                try:
                    gone = s.recv(1, socket.MSG_PEEK) == b""
                except BlockingIOError:
                    gone = False
                s.setblocking(True)
                if not gone:
                    return s, ready
            except (OSError, ConnectionError, ValueError, struct.error):
                pass
            try:
                s.close()
            except OSError:
                pass

    @staticmethod
    def _pump(fn, src, dst):
        try:
            fn(src, dst)
        except (ConnectionError, OSError, ValueError):
            pass
        finally:
            for s in (src, dst):
                try:
                    s.close()
                except OSError:
                    pass


def main():
    ap = argparse.ArgumentParser(description="Live Link bridge")
    ap.add_argument("--unity-port", type=int, default=47600, help="the Live Link's port (Active Port in Unity)")
    ap.add_argument("--stack-port", type=int, default=0, help="port between this script and Octave (default: Unity's + 100)")
    ap.add_argument("--repo", default=os.path.dirname(os.path.abspath(__file__)), help="repository root")
    ap.add_argument("--octave", default="", help="octave-cli executable")
    args = ap.parse_args()
    stack_port = args.stack_port or args.unity_port + 100
    octave = args.octave or find_octave()
    if not octave:
        sys.exit("main.py: octave-cli not found; pass --octave")
    print(f"main.py: starting the stack ({octave}) on port {stack_port}", flush=True)
    proc = start_stack(octave, args.repo, stack_port)
    try:
        Bridge(args.unity_port, stack_port).serve()
    finally:
        proc.terminate()


if __name__ == "__main__":
    main()
