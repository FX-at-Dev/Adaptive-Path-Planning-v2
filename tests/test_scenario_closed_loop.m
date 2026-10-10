function test_scenario_closed_loop()





















[scn, cfg] = ap_scn_village_road();
cfg.verbose = false;

[result, log] = ap_run_scenario(scn, cfg, ...
                                 struct('verbose', false, 'snapshots', false));
m = ap_metrics(result, log, cfg);


ap_assert_true(~result.collided, ...
    'the vehicle collided at t = %.2f s (min clearance %.3f m)', ...
    result.collide_t, result.min_clear);

ap_assert_true(m.min_clear > 0.15, ...
    'minimum clearance %.3f m is too tight', m.min_clear);

ap_assert_true(m.t_near_miss < 1.0, ...
    'spent %.2f s inside the near-miss threshold', m.t_near_miss);


ap_assert_true(result.reached, ...
    'did not reach the goal within %.0f s (%.0f m travelled, %.1f m short)', ...
    cfg.sim.t_end, m.dist_travel, log.d_goal(end));

ap_assert_true(m.t_to_goal < 0.9 * cfg.sim.t_end, ...
    'reached the goal at %.1f s, uncomfortably close to the %.0f s limit', ...
    m.t_to_goal, cfg.sim.t_end);

ap_assert_true(m.dist_travel > 150, ...
    'only travelled %.0f m; the route is about 190 m', m.dist_travel);





ap_assert_true(m.v_mean > 2.5, ...
    'mean speed %.2f m/s is too timid for an 8 m/s limit', m.v_mean);


ap_assert_true(m.latency_p95_ms < 150, ...
    'p95 replanning latency %.1f ms', m.latency_p95_ms);

ap_assert_true(m.latency_mean_ms < cfg.metric.latency_budget, ...
    'mean replanning latency %.1f ms exceeds the %.0f ms budget', ...
    m.latency_mean_ms, cfg.metric.latency_budget);

ap_assert_true(m.n_replans > 300, ...
    'only %d planner cycles ran; the loop is not replanning as expected', ...
    m.n_replans);


ap_assert_true(m.curv_max <= cfg.metric.curv_limit * 1.2, ...
    'peak path curvature %.3f 1/m exceeds the steering envelope', m.curv_max);

ap_assert_true(m.frac_jerk_ok > 0.75, ...
    'only %.0f%% of moving samples are within the jerk comfort limit', ...
    100 * m.frac_jerk_ok);




i_cruise = find(strcmp(m.state_names, 'CRUISE'));
ap_assert_true(m.state_frac(i_cruise) < 0.9, ...
    'spent %.0f%% of the run in CRUISE; the scenario is not challenging the planner', ...
    100 * m.state_frac(i_cruise));

ap_assert_true(sum(m.state_frac) > 0.99, ...
    'behaviour states account for only %.2f of the run', sum(m.state_frac));


ap_assert_true(m.tracks_mean > 0.5, ...
    'mean confirmed track count %.2f -- perception is not seeing the traffic', ...
    m.tracks_mean);
end
