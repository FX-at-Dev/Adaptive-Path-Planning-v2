function [a_cmd, delta_cmd, ctrl] = ap_controller(ego, traj, ctrl, cfg)






















if ~isfield(ctrl, 'e_int') || isempty(ctrl.e_int)
    ctrl.e_int = 0;
end

dt = cfg.sim.dt;


if isempty(traj) || numel(traj.t) < 2
    v_ref = 0;
    a_ff  = cfg.veh.a_min;
else






    t_look = cfg.ctrl.speed_lookahead * max(traj.T, 1.0);
    t_look = min(max(t_look, 0.4), traj.t(end));

    v_ref  = interp1(traj.t, traj.v, t_look, 'linear');
    if isfield(traj, 'a')
        a_ff = interp1(traj.t, traj.a, t_look, 'linear');
    else
        a_ff = 0;
    end








    if isfield(traj, 'v_target') && traj.v_target > ego.v
        v_ref = max(v_ref, min(ego.v, traj.v_target));
    end
end

e = v_ref - ego.v;
ctrl.e_int = ctrl.e_int + e * dt;
ctrl.e_int = min(max(ctrl.e_int, -cfg.ctrl.i_clamp), cfg.ctrl.i_clamp);

a_cmd = a_ff + cfg.ctrl.kp_speed * e + cfg.ctrl.ki_speed * ctrl.e_int;
a_cmd = min(max(a_cmd, cfg.veh.a_emergency), cfg.veh.a_max);







if ~isfield(ctrl, 'a_prev') || isempty(ctrl.a_prev)
    ctrl.a_prev = 0;
end










if ego.v <= 0 && a_cmd < 0
    a_cmd = 0;
end

da_raw = a_cmd - ctrl.a_prev;
if da_raw >= 0
    rate = cfg.veh.jerk_max;
else
    rate = cfg.veh.jerk_emergency;
end
da    = min(max(da_raw, -rate * dt), rate * dt);
a_cmd = ctrl.a_prev + da;

ctrl.a_prev = a_cmd;
ctrl.v_ref  = v_ref;
ctrl.a_ff   = a_ff;


Ld = max(cfg.ctrl.lookahead_min, cfg.ctrl.lookahead_gain * ego.v);

if isempty(traj) || numel(traj.x) < 2


    delta_cmd  = 0;
    ctrl.Ld    = Ld;
    ctrl.alpha = 0;
    return;
end

px = traj.x(:);
py = traj.y(:);



dist = sqrt((px - ego.x).^2 + (py - ego.y).^2);
idx  = find(dist >= Ld, 1, 'first');
if isempty(idx)
    idx = numel(px);
end


alpha  = ap_wrap_pi(atan2(py(idx) - ego.y, px(idx) - ego.x) - ego.psi);
Ld_eff = max(dist(idx), 1e-3);

delta_cmd = atan2(2 * cfg.ego.wheelbase * sin(alpha), Ld_eff);
delta_cmd = min(max(delta_cmd, -cfg.veh.delta_max), cfg.veh.delta_max);

ctrl.Ld    = Ld_eff;
ctrl.alpha = alpha;
end
