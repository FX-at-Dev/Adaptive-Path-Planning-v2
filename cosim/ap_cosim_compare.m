function ok = ap_cosim_compare(name, csv)













[scn, cfg] = feval(['ap_scn_' name]);
cfg.verbose = false;
[r, log] = ap_run_scenario(scn, cfg, struct('verbose', false, 'snapshots', false, 'think', true));

fid = fopen(csv, 'r');
if fid < 0
    error('ap_cosim_compare:read', 'cannot read %s', csv);
end
fgetl(fid);
C = textscan(fid, '%f %f %f %f %f %f %f %f %s %f %f', 'Delimiter', ',');
fclose(fid);
live.x = C{3}; live.y = C{4}; live.psi = C{5}; live.v = C{6};
live.state = C{9};
live.reached = C{10}(end) > 0.5;
live.collided = C{11}(end) > 0.5;

n = min(numel(live.x), numel(log.x));
dx   = max(abs(live.x(1:n) - log.x(1:n)));
dy   = max(abs(live.y(1:n) - log.y(1:n)));
dpsi = max(abs(ap_wrap_pi(live.psi(1:n) - log.psi(1:n))));
dv   = max(abs(live.v(1:n) - log.v(1:n)));
nstate = sum(~strcmp(live.state(1:n), log.state(1:n)));

pos = max(dx, dy);
ok = pos < 1e-9 && numel(live.x) == numel(log.x) && nstate == 0 && ...
     live.reached == r.reached && live.collided == r.collided;
fprintf('compare %s: steps live %d offline %d\n', name, numel(live.x), numel(log.x));
fprintf('compare max |dx| %.3g m, |dy| %.3g m, |dpsi| %.3g rad, |dv| %.3g m/s\n', dx, dy, dpsi, dv);
fprintf('compare behaviour states differing: %d of %d steps\n', nstate, n);
fprintf('compare reached live %d offline %d, collided live %d offline %d\n', ...
        live.reached, r.reached, live.collided, r.collided);
if ok
    fprintf('compare RESULT PASS: the live run reproduces the offline run (< 1e-9 m)\n');
else
    fprintf('compare RESULT FAIL\n');
end
end
