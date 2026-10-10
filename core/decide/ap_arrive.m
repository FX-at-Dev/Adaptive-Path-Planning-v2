function [bp, info] = ap_arrive(bp, ego, goal, rp, cfg)

















info.dist = Inf;
info.active = false;
if isempty(goal) || any(~isfinite(goal))
    return;
end
s_goal = ap_cart2frenet(rp, goal(1), goal(2));
s_ego = ap_cart2frenet(rp, ego.x + cos(ego.psi) * cfg.ego.rear_axle_to_centre, ...
                            ego.y + sin(ego.psi) * cfg.ego.rear_axle_to_centre);
info.dist = s_goal - s_ego;

d = max(info.dist - cfg.dec.arrive_lead, 0);
v_stop = sqrt(2 * cfg.dec.arrive_decel * d);
if info.dist <= cfg.dec.arrive_tol
    v_stop = 0;
end
if v_stop < bp.v_cap
    bp.v_cap = v_stop;
    info.active = true;
    if v_stop == 0
        bp.reason = sprintf('at the target (%.1f m)', info.dist);
    else
        bp.reason = sprintf('%s; stopping at the target, %.0f m ahead', bp.reason, info.dist);
    end
end
end
