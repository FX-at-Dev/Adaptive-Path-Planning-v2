function [reached, dist] = ap_goal_reached(ego, goal, rp, cfg)






s_goal = ap_cart2frenet(rp, goal(1), goal(2));
s_ego = ap_cart2frenet(rp, ego.x + cos(ego.psi) * cfg.ego.rear_axle_to_centre, ...
                            ego.y + sin(ego.psi) * cfg.ego.rear_axle_to_centre);
dist = s_goal - s_ego;



reached = dist <= cfg.dec.goal_tol && ego.v < cfg.dec.arrive_v;
end
