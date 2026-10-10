






using System.Diagnostics;
using System.Runtime.InteropServices;

namespace Ap
{
    public static class ApProcess
    {
#if UNITY_EDITOR_WIN || UNITY_STANDALONE_WIN
        [StructLayout(LayoutKind.Sequential)]
        struct PowerThrottling { public uint Version, ControlMask, StateMask; }

        [DllImport("kernel32.dll", SetLastError = true)]
        static extern bool SetProcessInformation(System.IntPtr process, int infoClass, ref PowerThrottling info, int size);

        const int ProcessPowerThrottling = 4;
        const uint ExecutionSpeed = 1;
#endif



        public static bool RunAtFullSpeed(Process p)
        {
            if (p == null) return false;
            bool ok = false;
#if UNITY_EDITOR_WIN || UNITY_STANDALONE_WIN
            try
            {
                var s = new PowerThrottling { Version = 1, ControlMask = ExecutionSpeed, StateMask = 0 };
                ok = SetProcessInformation(p.Handle, ProcessPowerThrottling, ref s, Marshal.SizeOf(typeof(PowerThrottling)));
            }
            catch { ok = false; }
#endif
            try { p.PriorityClass = ProcessPriorityClass.AboveNormal; } catch { }
            return ok;
        }
    }
}
