function [scn, cfg] = ap_scn_market(cfg, seed)

























if nargin < 1 || isempty(cfg)
    cfg = ap_config();
end

cfg.plan.v_max              = 5.0;
cfg.plan.corridor_halfwidth = 4.0;
cfg.sim.t_end               = 140.0;


cfg.dec.creep_speed         = 1.6;
cfg.dec.clear_caution       = 2.0;
cfg.plan.safety_margin      = 0.35;



cfg.pred.horizon            = 3.0;
cfg.plan.horizon_T          = [2.0 3.0];


road_wp = [  0,   0;
            30,   3;
            60,   4;
            95,  -1;
           130,  -4;
           160,  -3];

scn.name = 'market';
scn.desc = 'Dense market street, stalls and pushcarts narrowing the road, pedestrians crossing';
scn.rp   = ap_ref_path(road_wp, 0.25, cfg.plan.corridor_halfwidth);

scn.ego  = struct('x', 0, 'y', 0, 'psi', atan2(3, 30), 'v', 3.0);
[gx, gy] = ap_wp_at(scn.rp, scn.rp.length - 10, 0);
scn.goal = [gx, gy];
























statics = [ 30, -3.4, 1;
            70,  3.4, 2;
           112, -3.4, 1];

agents = [];
id = 0;
for k = 1:size(statics, 1)
    id = id + 1;
    if statics(k,3) == 1
        cls = 'pushcart';
    else
        cls = 'static';
    end
    agents = [agents, local_parked(id, cls, scn.rp, statics(k,1), statics(k,2))];
end










id = id + 1;



agents = [agents, local_walker(id, 'pedestrian', scn.rp,  48, -4.8,  4.8, 1.0,  6.0)];
id = id + 1;
agents = [agents, local_walker(id, 'pedestrian', scn.rp,  88,  4.8, -4.8, 1.1, 20.0)];
id = id + 1;
agents = [agents, local_walker(id, 'pedestrian', scn.rp, 126, -4.8,  4.8, 0.9, 40.0)];
id = id + 1;
agents = [agents, local_walker(id, 'pedestrian', scn.rp, 152,  4.8, -4.8, 1.2, 58.0)];


id = id + 1;
cyc = ap_wp_path(scn.rp, [ 45  70  95 120 150], [1.0 -0.6 0.8 -0.4 0.4]);
agents = [agents, local_mover(id, 'bicycle', scn.rp, 45, 1.0, 2.4, cyc, 0)];


id = id + 1;





onc = ap_wp_path(scn.rp, [150 120  90  60  30  -5], [-1.9 -1.1 -2.1 -1.3 -1.8 -1.4]);
agents = [agents, local_mover(id, 'auto', scn.rp, 150, -0.8, 3.2, onc, 34.0)];

scn.agents = agents;
if nargin >= 2 && ~isempty(seed)
    [scn, cfg] = ap_scn_random(scn, cfg, seed, @local_draw);
end
end


function a = local_parked(id, class_name, rp, s_at, d_at)
[x0, y0] = ap_wp_at(rp, s_at, d_at);
a = ap_agent_new(id, class_name, x0, y0, ap_wp_heading(rp, s_at), 0.0, 'static', [], 0);
end


function a = local_walker(id, class_name, rp, s_at, d_from, d_to, v, t_spawn)
[x0, y0] = ap_wp_at(rp, s_at, d_from);
wp   = ap_wp_path(rp, [s_at, s_at + 1, s_at + 2], [d_from, 0, d_to]);
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
if size(wp,1) >= 2
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
s = R.u(25, 35); side = R.pick({1, -1});
while s < L - 25
    id = id + 1;
    A = [A, ap_role('parked', id, R.pick({'pushcart', 'static', 'pushcart'}), rp, s, side * R.u(3.2, 3.5))];
    side = -side;
    s = s + R.u(25, 42);
end
for k = 1:R.int(3, 5)
    id = id + 1;
    sp = R.u(40, L - 8); side = R.pick({1, -1});
    A = [A, ap_role('walker', id, 'pedestrian', rp, sp, side * 4.8, -side * 4.8, R.u(0.9, 1.2), ...
                     max(0, sp / 2.6 - R.u(6, 12)))];
end
id = id + 1;
s0 = R.u(35, 60); sl = s0 + 25:25:L; dl = arrayfun(@(~) R.u(-0.6, 1.0), sl);
A = [A, ap_role('mover', id, 'bicycle', rp, s0, R.u(0.4, 1.2), R.u(2.0, 2.8), sl, dl, 0)];
id = id + 1;
s0 = R.u(130, L - 5); sl = s0 - 30:-30:-5; dl = arrayfun(@(~) R.u(-2.2, -1.0), sl);
A = [A, ap_role('mover', id, 'auto', rp, s0, -0.8, R.u(2.8, 3.6), sl, dl, R.u(25, 45))];
end
