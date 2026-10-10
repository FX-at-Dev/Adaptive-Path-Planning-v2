function [scn, cfg] = ap_scn_urban_intersection(cfg, seed)





















if nargin < 1 || isempty(cfg)
    cfg = ap_config();
end

cfg.plan.v_max              = 10.0;
cfg.plan.corridor_halfwidth = 4.0;
cfg.sim.t_end               = 75.0;
cfg.dec.yield_gap           = 4.5;


scn.name = 'urban_intersection';
scn.desc = 'Unsignalled urban crossroads, cross traffic that does not yield';
scn.rp   = ap_ref_path([0 0; 60 0; 120 0; 170 0], 0.25, cfg.plan.corridor_halfwidth);

XJ = 85;


scn.cross_streets = [XJ, 7.0];

scn.ego  = struct('x', 0, 'y', 0, 'psi', 0, 'v', 8.0);
[gx, gy] = ap_wp_at(scn.rp, scn.rp.length - 12, 0);
scn.goal = [gx, gy];





scn.agents = [ ...
    local_crosser(1, 'auto',        scn.rp, XJ -  6, -52,  70,  6.2), ...
    local_crosser(2, 'car',         scn.rp, XJ +  7,  46, -70,  7.5), ...
    local_crosser(3, 'two_wheeler', scn.rp, XJ +  1, -64,  70,  9.0), ...
    local_crosser(4, 'auto',        scn.rp, XJ + 13,  58, -70,  5.5), ...
    local_walker(5, 'pedestrian', scn.rp, XJ + 26, 5.5, -5.5, 1.2, 9.0), ...
    local_parked(6, 'bus', scn.rp, XJ + 46, -3.4)];
if nargin >= 2 && ~isempty(seed)
    [scn, cfg] = ap_scn_random(scn, cfg, seed, @local_draw);
end
end


function a = local_crosser(id, class_name, rp, s_at, d_from, d_to, v)






[x0, y0] = ap_wp_at(rp, s_at, d_from);



wp = ap_wp_path(rp, [s_at, s_at, s_at], [d_from, 0, d_to]);



psi0 = ap_wp_heading(rp, s_at) + sign(d_to - d_from) * pi/2;

a = ap_agent_new(id, class_name, x0, y0, psi0, abs(v), 'path', wp, 0);
end


function a = local_walker(id, class_name, rp, s_at, d_from, d_to, v, t_spawn)

[x0, y0] = ap_wp_at(rp, s_at, d_from);
wp   = ap_wp_path(rp, [s_at, s_at + 1, s_at + 2], [d_from, 0, d_to]);
psi0 = ap_wp_heading(rp, s_at) + sign(d_to - d_from) * pi/2;
a = ap_agent_new(id, class_name, x0, y0, psi0, v, 'path', wp, t_spawn);
end


function a = local_parked(id, class_name, rp, s_at, d_at)

[x0, y0] = ap_wp_at(rp, s_at, d_at);
a = ap_agent_new(id, class_name, x0, y0, ap_wp_heading(rp, s_at), 0.0, 'static', [], 0);
end


function A = local_draw(rp, cfg, R)


XJ = 85; A = []; id = 0;
speed = struct('auto', [5.0 6.5], 'car', [6.5 8.5], 'two_wheeler', [8.0 10.0], 'bus', [5.0 6.5]);
for k = 1:R.int(3, 5)
    id = id + 1;
    cls = R.pick({'auto', 'car', 'two_wheeler', 'auto', 'car', 'bus'});
    dir = R.pick({1, -1});
    A = [A, ap_role('cross_traffic', id, cls, rp, XJ + R.u(-7, 14), -dir * R.u(45, 65), dir * 70, ...
                     R.u(speed.(cls)(1), speed.(cls)(2)))];
end
id = id + 1;
side = R.pick({1, -1});
A = [A, ap_role('walker', id, 'pedestrian', rp, XJ + R.u(20, 32), side * 5.5, -side * 5.5, R.u(1.0, 1.3), R.u(6, 11))];
id = id + 1;
A = [A, ap_role('parked', id, R.pick({'bus', 'truck', 'car'}), rp, XJ + R.u(38, 55), R.pick({1, -1}) * 3.4)];
end
