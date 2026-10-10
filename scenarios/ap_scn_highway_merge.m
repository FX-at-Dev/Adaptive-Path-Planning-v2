function [scn, cfg] = ap_scn_highway_merge(cfg, seed)





















if nargin < 1 || isempty(cfg)
    cfg = ap_config();
end

cfg.plan.v_max              = 18.0;
cfg.plan.corridor_halfwidth = 3.6;
cfg.sim.t_end               = 75.0;


cfg.pred.horizon            = 5.0;
cfg.plan.horizon_T          = [3.0 5.0];





road_wp = [   0,  -7.0;
             50,  -6.6;
            100,  -3.4;
            150,  -0.6;
            220,   0.0;
            300,   0.0];

scn.name = 'highway_merge';
scn.desc = 'Slip-road merge into slow heavy traffic, no gap conceded';
scn.rp   = ap_ref_path(road_wp, 0.25, cfg.plan.corridor_halfwidth);

scn.ego  = struct('x', 0, 'y', -7.0, 'psi', atan2(0.4, 50), 'v', 14.0);
[gx, gy] = ap_wp_at(scn.rp, scn.rp.length - 15, 0);
scn.goal = [gx, gy];





lane   = @(s0, d) ap_wp_path(scn.rp, s0, d);

truck_wp = lane([ 95 150 210 270 320], [ 0.2  0.0 -0.2  0.0  0.0]);
bus_wp   = lane([140 200 260 320],     [-0.4 -0.2  0.0  0.0]);
tw1_wp   = lane([ 60 120 180 240 320], [ 2.4  2.0  2.6  2.2  2.0]);
tw2_wp   = lane([ 30  90 150 220 320], [-2.6 -2.2 -2.8 -2.4 -2.0]);
car_wp   = lane([ 10  70 130 200 320], [ 1.0  0.6  1.2  0.8  0.6]);

scn.agents = [ ...
    local_mover(1, 'truck',       scn.rp,  95,  0.2, 10.0, truck_wp), ...
    local_mover(2, 'bus',         scn.rp, 140, -0.4,  8.5, bus_wp), ...
    local_mover(3, 'two_wheeler', scn.rp,  60,  2.4, 16.0, tw1_wp), ...
    local_mover(4, 'two_wheeler', scn.rp,  30, -2.6, 17.0, tw2_wp), ...
    local_mover(5, 'car',         scn.rp,  10,  1.0, 15.5, car_wp)];
if nargin >= 2 && ~isempty(seed)
    [scn, cfg] = ap_scn_random(scn, cfg, seed, @local_draw);
end
end


function a = local_mover(id, class_name, rp, s0, d0, v, wp)

[x0, y0] = ap_wp_at(rp, s0, d0);
a = ap_agent_new(id, class_name, x0, y0, ap_wp_heading(rp, s0), v, 'path', wp, 0);
end


function A = local_draw(rp, cfg, R)


L = rp.length; A = []; id = 0;
s_heavy = R.u(85, 150);
for k = 1:1 + R.coin(0.6)
    id = id + 1;
    s0 = s_heavy + (k - 1) * R.u(40, 60); d0 = R.u(-0.5, 0.3);
    sl = s0 + 60:60:L + 20; dl = arrayfun(@(~) R.u(-0.4, 0.2), sl);
    A = [A, ap_role('mover', id, R.pick({'truck', 'bus'}), rp, s0, d0, R.u(8, 10.5), sl, dl, 0)]; %#ok<AGROW>
end
for k = 1:R.int(1, 3)
    id = id + 1;
    side = R.pick({1, -1});
    s0 = R.u(20, 80); d0 = side * R.u(1.8, 2.8);
    sl = s0 + 60:60:L + 20; dl = side * arrayfun(@(~) R.u(1.8, 2.8), sl);
    A = [A, ap_role('mover', id, 'two_wheeler', rp, s0, d0, R.u(14, 17.5), sl, dl, 0)];
end
id = id + 1;
s0 = R.u(10, 25); d0 = R.u(0.6, 1.4);
sl = s0 + 60:60:L + 20; dl = arrayfun(@(~) R.u(0.6, 1.2), sl);
A = [A, ap_role('mover', id, 'car', rp, s0, d0, R.u(14, 16), sl, dl, 0)];
end
