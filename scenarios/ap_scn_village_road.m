function [scn, cfg] = ap_scn_village_road(cfg, seed)





























if nargin < 1 || isempty(cfg)
    cfg = ap_config();
end


cfg.plan.v_max              = 8.0;
cfg.plan.corridor_halfwidth = 3.2;



cfg.sim.t_end               = 120.0;




road_wp = [   0,   0;
             40,   2;
             80,  10;
            120,  16;
            160,  14;
            200,   6];

scn.name = 'village_road';
scn.rp   = ap_ref_path(road_wp, 0.25, cfg.plan.corridor_halfwidth);
scn.desc = 'Unmarked village road, oncoming rickshaw, cyclist, verge pedestrian, parked pushcart';


scn.ego = struct('x', 0, 'y', 0, 'psi', atan2(2, 40), 'v', 6.0);



gx = interp1(scn.rp.s, scn.rp.x, scn.rp.length - 12);
gy = interp1(scn.rp.s, scn.rp.y, scn.rp.length - 12);
scn.goal = [gx, gy];






onc = local_offset_path(scn.rp, [190 150 110 70 30 -10], [-1.4 -1.1 -1.7 -1.2 -1.5 -1.3]);
cyc = local_offset_path(scn.rp, [ 55  95 135 175 210],   [ 1.6  1.9 1.4 1.0 1.0]);



ped = local_offset_path(scn.rp, [ 70  76  84  92],      [ 3.4  1.2 -1.8 -4.6]);

[px, py] = local_point(scn.rp, 118, -2.4);
[ax, ay] = local_point(scn.rp, 190, -1.4);
[bx, by] = local_point(scn.rp,  55,  1.6);
[qx, qy] = local_point(scn.rp,  70,  3.0);

scn.agents = [ ...
    ap_agent_new(1, 'auto',       ax, ay, local_heading(scn.rp, 190) + pi, ...
                  6.5, 'path', onc, 0), ...
    ap_agent_new(2, 'bicycle',    bx, by, local_heading(scn.rp,  55), ...
                  3.6, 'path', cyc, 0), ...
    ap_agent_new(3, 'pedestrian', qx, qy, local_heading(scn.rp,  70) - 0.9, ...
                  1.1, 'path', ped, 6.0), ...
    ap_agent_new(4, 'pushcart',   px, py, local_heading(scn.rp, 118), ...
                  0.0, 'static', [], 0)];
if nargin >= 2 && ~isempty(seed)
    [scn, cfg] = ap_scn_random(scn, cfg, seed, @local_draw);
end
end


function wp = local_offset_path(rp, s_list, d_list)

wp = zeros(numel(s_list), 2);
for k = 1:numel(s_list)
    [wp(k,1), wp(k,2)] = local_point(rp, s_list(k), d_list(k));
end
end


function [x, y] = local_point(rp, s, d)
s = min(max(s, 0), rp.length);
[x, y] = ap_frenet2cart(rp, s, d);
end


function psi = local_heading(rp, s)
s   = min(max(s, 0), rp.length);
psi = ap_wrap_pi(interp1(rp.s, rp.psi, s, 'linear'));
end


function A = local_draw(rp, cfg, R)


L = rp.length; A = []; id = 0;
speed = struct('auto', [5.5 7.0], 'car', [6.0 7.5], 'two_wheeler', [6.5 8.0], 'bicycle', [3.0 4.0]);
for k = 1:R.int(1, 2)
    id = id + 1;
    cls = R.pick({'auto', 'car', 'two_wheeler', 'auto', 'bicycle'});
    s0 = R.u(130, L - 5); d0 = R.u(-2.0, -0.9);
    sl = s0 - 40:-40:-20; dl = d0 + arrayfun(@(~) R.u(-0.6, 0.6), sl);
    A = [A, ap_role('mover', id, cls, rp, s0, d0, R.u(speed.(cls)(1), speed.(cls)(2)), ...
                     sl, max(min(dl, -0.8), -2.1), (k - 1) * R.u(8, 15))];
end
id = id + 1;
cls = R.pick({'bicycle', 'bicycle', 'two_wheeler'});
s0 = R.u(40, 80); d0 = R.u(1.0, 2.0);
sl = s0 + 40:40:L + 20; dl = arrayfun(@(~) R.u(0.8, 2.0), sl);
A = [A, ap_role('mover', id, cls, rp, s0, d0, R.u(speed.(cls)(1), speed.(cls)(2)) * 0.8, sl, dl, 0)];
id = id + 1;
s = R.u(60, 120); side = R.pick({1, -1});
A = [A, ap_role('walker', id, 'pedestrian', rp, s, side * R.u(3.4, 4.2), -side * R.u(3.6, 4.8), ...
                 R.u(0.9, 1.3), max(0, s / 6 - R.u(5, 8)))];
for k = 1:R.int(1, 2)
    id = id + 1;
    cls = R.pick({'pushcart', 'static', 'car'});
    A = [A, ap_role('parked', id, cls, rp, R.u(95, 165), R.pick({1, -1}) * R.u(2.2, 2.6))]; %#ok<AGROW>
end
end
