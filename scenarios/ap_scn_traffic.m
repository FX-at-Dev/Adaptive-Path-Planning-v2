function A = ap_scn_traffic(scn, cfg, R, id0)




















rp = scn.rp;
L = rp.length;
hw = cfg.plan.corridor_halfwidth;
lane = ap_lane_centre(hw, cfg);
two_lane = lane > 0;
s_ego = ap_cart2frenet(rp, scn.ego.x, scn.ego.y);
s_goal = min(ap_cart2frenet(rp, scn.goal(1), scn.goal(2)), L);

n = local_counts(scn.name);
A = [];
id = id0;


for k = 1:R.int(n.oncoming(1), n.oncoming(2))
    cls = R.pick({'car', 'auto', 'auto', 'two_wheeler', 'two_wheeler', 'bus', 'truck', 'bicycle'});
    p = ap_agent_props(cls);
    v = p.v_typ * R.u(0.7, 1.05);
    s0 = R.u(s_ego + 45, min(L, s_goal + 60));
    d0 = local_fit(-lane + R.u(-0.4, 0.4) * two_lane, p, hw);
    sl = [s0 - 30:-30:-25, -25];
    dl = arrayfun(@(~) local_fit(d0 + R.u(-0.3, 0.3), p, hw), sl);
    A = [A, ap_role('mover', id, cls, rp, s0, d0, v, sl, dl, 0)]; %#ok<AGROW>
    id = id + 1;
end


for k = 1:R.int(n.ahead(1), n.ahead(2))
    cls = R.pick({'bicycle', 'bicycle', 'two_wheeler', 'auto', 'pushcart'});
    p = ap_agent_props(cls);
    v = max(0.8, p.v_typ * R.u(0.5, 0.9));
    s0 = R.u(s_ego + 25, max(s_ego + 30, s_goal - 25));
    d0 = local_fit(hw - p.width / 2 - R.u(0.3, 0.8), p, hw);
    sl = [s0 + 30:30:L + 25, L + 25];
    dl = arrayfun(@(~) local_fit(d0 + R.u(-0.25, 0.25), p, hw), sl);
    A = [A, ap_role('mover', id, cls, rp, s0, d0, v, sl, dl, 0)]; %#ok<AGROW>
    id = id + 1;
end


for k = 1:R.int(n.parked(1), n.parked(2))
    cls = R.pick({'car', 'auto', 'pushcart', 'two_wheeler', 'truck'});
    p = ap_agent_props(cls);
    side = R.pick({1, -1});
    s = R.u(s_ego + 25, max(s_ego + 30, s_goal - 12));
    A = [A, ap_role('parked', id, cls, rp, s, side * (hw - p.width / 2 - R.u(0.0, 0.3)))]; %#ok<AGROW>
    id = id + 1;
end


for k = 1:R.int(n.walkers(1), n.walkers(2))
    side = R.pick({1, -1});
    d0 = side * (hw + R.u(0.4, 1.3));
    s0 = R.u(s_ego + 15, min(L, s_goal + 20));
    v = R.u(0.9, 1.4);
    if R.coin(0.5), sl = [s0 + 25:25:L + 20, L + 20]; else, sl = [s0 - 25:-25:-20, -20]; end
    dl = arrayfun(@(~) d0 + R.u(-0.2, 0.2), sl);
    A = [A, ap_role('mover', id, 'pedestrian', rp, s0, d0, v, sl, dl, 0)];
    id = id + 1;
end


for k = 1:R.int(n.cattle(1), n.cattle(2))
    side = R.pick({1, -1});
    s = R.u(s_ego + 30, max(s_ego + 35, s_goal - 10));
    if R.coin(0.35)


        t_go = max(0, (s - s_ego) / 7.0 - R.u(3, 7));
        A = [A, ap_role('crosser', id, 'cattle', rp, s, side * (hw + R.u(1.0, 2.0)), ...
                         -side * (hw + R.u(1.5, 3.0)), R.u(0.6, 1.0), t_go)];
    else
        a = ap_role('parked', id, 'cattle', rp, s, side * (hw + R.u(0.8, 2.5)));
        a.psi = a.psi + R.u(-1.2, 1.2);
        A = [A, a];
    end
    id = id + 1;
end
end


function n = local_counts(name)

n = struct('oncoming', [1 3], 'ahead', [0 2], 'parked', [1 2], 'walkers', [1 3], 'cattle', [0 2]);
switch name
    case 'highway_merge'      % one-way carriageway: no oncoming, no one on foot
        n = struct('oncoming', [0 0], 'ahead', [1 2], 'parked', [0 1], 'walkers', [0 0], 'cattle', [0 0]);
    case 'market'             % already crowded by its story: mostly people
        n = struct('oncoming', [0 1], 'ahead', [0 1], 'parked', [0 1], 'walkers', [2 4], 'cattle', [0 1]);
    case 'urban_intersection'
        n = struct('oncoming', [1 3], 'ahead', [0 2], 'parked', [1 2], 'walkers', [1 3], 'cattle', [0 1]);
    case 'village_road'       % a narrow road: room for its story and a little more
        n = struct('oncoming', [1 2], 'ahead', [0 1], 'parked', [0 1], 'walkers', [1 2], 'cattle', [0 1]);
    case 'cattle_crossing'
        n = struct('oncoming', [1 2], 'ahead', [0 1], 'parked', [0 1], 'walkers', [0 2], 'cattle', [1 3]);
end
end


function d = local_fit(d, p, hw)

lim = max(hw - p.width / 2 - 0.15, 0);
d = min(max(d, -lim), lim);
end
