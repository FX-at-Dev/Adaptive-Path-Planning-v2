function [scn, cfg] = ap_scn_cattle_crossing(cfg, seed)



























if nargin < 1 || isempty(cfg)
    cfg = ap_config();
end

cfg.plan.v_max              = 12.0;
cfg.plan.corridor_halfwidth = 3.7;
cfg.sim.t_end               = 140.0;




cfg.pred.horizon            = 5.0;
cfg.plan.horizon_T          = [3.0 5.0];


road_wp = [   0,   0;
             50,  -2;
            100,  -3;
            150,   0;
            200,   4;
            240,   5];

scn.name = 'cattle_crossing';
scn.desc = 'Open rural road, cattle emerging from behind a parked truck at speed';
scn.rp   = ap_ref_path(road_wp, 0.25, cfg.plan.corridor_halfwidth);

scn.ego  = struct('x', 0, 'y', 0, 'psi', atan2(-2, 50), 'v', 11.0);
[gx, gy] = ap_wp_at(scn.rp, scn.rp.length - 12, 0);
scn.goal = [gx, gy];





S_CROSS = 105;




truck = local_parked(1, 'truck', scn.rp, S_CROSS - 13, 3.1);





cow1 = local_crosser(2, 'cattle', scn.rp, S_CROSS - 2, 5.0, -5.4, 1.0,  7.5);
cow2 = local_crosser(3, 'cattle', scn.rp, S_CROSS + 3, 5.6, -5.2, 0.8,  8.6);
cow3 = local_crosser(4, 'cattle', scn.rp, S_CROSS + 9, 6.0, -5.0, 0.9, 10.4);




auto_wp = ap_wp_path(scn.rp, [ 60 110 160 210 250], [ 1.2  0.6  1.0  0.4  0.8]);
tw_wp   = ap_wp_path(scn.rp, [230 180 130  80  30  -10], [-1.0 -0.6 -1.2 -0.8 -1.0 -0.6]);

auto = local_mover(5, 'auto',        scn.rp,  60,  1.2, 7.0, auto_wp, 0);









tw   = local_mover(6, 'two_wheeler', scn.rp, 238, -1.8, 7.0, tw_wp,  38.0);

scn.agents = [truck, cow1, cow2, cow3, auto, tw];
if nargin >= 2 && ~isempty(seed)
    [scn, cfg] = ap_scn_random(scn, cfg, seed, @local_draw);
end
end


function a = local_parked(id, class_name, rp, s_at, d_at)
[x0, y0] = ap_wp_at(rp, s_at, d_at);
a = ap_agent_new(id, class_name, x0, y0, ap_wp_heading(rp, s_at), 0.0, 'static', [], 0);
end


function a = local_crosser(id, class_name, rp, s_at, d_from, d_to, v, t_spawn)




[x0, y0] = ap_wp_at(rp, s_at, d_from);
wp   = ap_wp_path(rp, [s_at, s_at + 2, s_at + 1, s_at + 3], ...
                       [d_from, 0.5 * d_from, 0.4 * d_to, d_to]);
psi0 = ap_wp_heading(rp, s_at) + sign(d_to - d_from) * pi/2;
a = ap_agent_new(id, class_name, x0, y0, psi0, v, 'path', wp, t_spawn);
end


function a = local_mover(id, class_name, rp, s0, d0, v, wp, t_spawn)

if t_spawn > 0 && size(wp, 1) >= 2
    s_end = ap_cart2frenet(rp, wp(end, 1), wp(end, 2));
    [s0, t_spawn] = ap_upstream(rp, s0, v, t_spawn, sign(s_end - s0));
end
[x0, y0] = ap_wp_at(rp, s0, d0);
psi0 = ap_wp_heading(rp, s0);
if size(wp, 1) >= 2
    dxw = wp(2,1) - x0;
    dyw = wp(2,2) - y0;
    if hypot(dxw, dyw) > 1e-6
        psi0 = atan2(dyw, dxw);
    end
end
a = ap_agent_new(id, class_name, x0, y0, psi0, v, 'path', wp, t_spawn);
end


function A = local_draw(rp, cfg, R)


L = rp.length; A = []; id = 0;
S = R.u(90, 125); side = R.pick({1, -1});
id = id + 1;
A = [A, ap_role('parked', id, R.pick({'truck', 'bus'}), rp, S - R.u(10, 16), side * 3.1)];
off = 0;
for k = 1:R.int(2, 4)
    id = id + 1;
    off = off + R.u(2, 6);
    A = [A, ap_role('crosser', id, 'cattle', rp, S - 2 + off, side * R.u(5.0, 6.2), -side * R.u(5.0, 5.6), ...
                     R.u(0.7, 1.1), max(0, S / 9.5 - R.u(1, 3.5) + 0.3 * off))];
end
id = id + 1;
s0 = R.u(45, 75); sl = s0 + 50:50:L + 10; dl = arrayfun(@(~) R.u(0.4, 1.2), sl);
A = [A, ap_role('mover', id, 'auto', rp, s0, R.u(0.6, 1.4), R.u(6, 8), sl, dl, 0)];
id = id + 1;
s0 = L - R.u(0, 10); sl = s0 - 50:-50:-10; dl = arrayfun(@(~) R.u(-1.2, -0.6), sl);
A = [A, ap_role('mover', id, 'two_wheeler', rp, s0, R.u(-1.9, -1.2), R.u(6.5, 8), sl, dl, R.u(30, 45))];
end
