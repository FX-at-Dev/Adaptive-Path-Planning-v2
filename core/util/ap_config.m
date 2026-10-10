function cfg = ap_config()










cfg.sim.dt          = 0.05;
cfg.sim.t_end       = 40.0;
cfg.sim.replan_dt   = 0.10;
cfg.scn.traffic     = true;
cfg.sim.seed        = 20260909;


cfg.ego.length      = 4.00;
cfg.ego.width       = 1.80;
cfg.ego.wheelbase   = 2.70;
cfg.ego.rear_axle_to_centre = 1.35;





cfg.ego.n_discs     = 3;


cfg.veh.a_max       =  2.50;
cfg.veh.a_min       = -4.50;
cfg.veh.a_emergency = -7.00;
cfg.veh.jerk_max       =  4.00;
cfg.veh.jerk_emergency = 25.00;
cfg.veh.delta_max   =  0.55;
cfg.veh.delta_rate  =  0.60;
cfg.veh.v_reverse_max = 1.5;
cfg.veh.a_lat_max   =  3.00;


cfg.ctrl.lookahead_min  = 3.0;
cfg.ctrl.lookahead_gain = 0.9;









cfg.ctrl.speed_lookahead = 0.28;
cfg.ctrl.kp_speed       = 1.6;
cfg.ctrl.ki_speed       = 0.20;
cfg.ctrl.i_clamp        = 2.0;




cfg.sensor.camera.range        = 60.0;
cfg.sensor.camera.fov          = deg2rad(100);
cfg.sensor.camera.sigma_range  = 0.08;
cfg.sensor.camera.sigma_bear   = deg2rad(0.5);
cfg.sensor.camera.pd           = 0.92;
cfg.sensor.camera.classifies   = true;
cfg.sensor.camera.class_acc    = 0.88;

cfg.sensor.radar.range         = 90.0;
cfg.sensor.radar.fov           = deg2rad(60);
cfg.sensor.radar.sigma_range   = 0.25;
cfg.sensor.radar.sigma_bear    = deg2rad(2.0);
cfg.sensor.radar.sigma_rate    = 0.10;
cfg.sensor.radar.pd            = 0.90;
cfg.sensor.radar.classifies    = false;

cfg.sensor.lidar.range         = 40.0;
cfg.sensor.lidar.fov           = 2*pi;
cfg.sensor.lidar.sigma_pos     = 0.06;
cfg.sensor.lidar.pd            = 0.95;
cfg.sensor.lidar.classifies    = false;


cfg.sensor.lidar.beams         = 32;
cfg.sensor.lidar.azimuth_bins  = 1024;
cfg.sensor.lidar.v_fov         = deg2rad([-15, 15]);
cfg.sensor.lidar.mount_height  = 1.90;

cfg.sensor.clutter_rate        = 0.6;
cfg.sensor.occlusion           = true;





cfg.sensor.lidar.dropout       = 0.05;
cfg.sensor.lidar.range_noise   = 0.02;
cfg.sensor.radar.mount_height  = 0.60;
cfg.sensor.radar.az_res        = deg2rad(1.0);
cfg.sensor.radar.v_fov         = deg2rad([-3, 3]);
cfg.sensor.camera.mount_height = 1.40;
cfg.sensor.camera.image_size   = [1280, 720];
cfg.sensor.camera.sigma_px     = 1.5;
cfg.sensor.camera.min_box_px   = 8;
cfg.sensor.camera.min_visible  = 0.2;






cfg.percep.ground_height   = 0.25;
cfg.percep.max_height      = 4.00;
cfg.percep.roi_near        = 1.2;
cfg.percep.roi_wide        = 10.0;
cfg.percep.wide_max_height = 2.2;
cfg.percep.wide_max_extent = 3.0;
cfg.percep.roi_cell        = 0.5;
cfg.percep.cell            = 0.3;
cfg.percep.join            = 0.3;

cfg.percep.min_points      = 4;
cfg.percep.fit_angles      = 30;
cfg.percep.fit_d0          = 0.05;
cfg.percep.fit_road_bias   = 0.15;
cfg.percep.max_extent      = 14.0;


cfg.percep.map_cell        = 0.25;
cfg.percep.map_margin      = 0.3;
cfg.percep.radar_map_margin = 2.5;
cfg.percep.radar_static_speed = 0.5;
cfg.percep.camera_edge_px  = 3;


cfg.fuse.gate_chi2   = 9.21;
cfg.fuse.confirm_M   = 3;
cfg.fuse.confirm_N   = 4;
cfg.fuse.max_misses  = 6;
cfg.fuse.max_misses_still = 20;
cfg.fuse.tentative_max_age = 20;
cfg.fuse.q_accel     = 1.5;
cfg.fuse.q_yawrate   = 0.6;
cfg.fuse.p0_pos      = 4.0;
cfg.fuse.p0_vel      = 9.0;
cfg.fuse.p0_yaw      = 1.0;
cfg.fuse.p0_omega    = 0.5;


cfg.pred.horizon     = 4.0;
cfg.pred.dt          = 0.20;
cfg.pred.n_hypo      = 4;
cfg.pred.sigma_grow  = 0.35;


cfg.risk.res         = 0.50;
cfg.risk.ahead       = 50.0;
cfg.risk.behind      = 10.0;
cfg.risk.lateral     = 20.0;
cfg.risk.inflate     = 0.40;


cfg.plan.v_max       = 8.0;
cfg.plan.v_min       = 0.0;





cfg.plan.horizon_T   = [2.5 4.0];
cfg.plan.d_step      = 0.55;
cfg.plan.v_samples   = 4;
cfg.plan.corridor_halfwidth = 3.0;
cfg.plan.safety_margin      = 0.45;






cfg.plan.max_path_slope     = 0.80;



cfg.plan.w_risk      = 60.0;
cfg.plan.w_jerk      = 0.8;
cfg.plan.w_curv      = 12.0;
cfg.plan.w_progress  = 6.0;
cfg.plan.w_offset    = 3.0;
cfg.plan.w_clearance = 10.0;
cfg.plan.w_speed_dev = 2.0;




cfg.plan.keep_left      = true;
cfg.plan.two_lane_width = 5.0;
cfg.plan.road_overhang  = 0.3;

cfg.plan.risk_threshold = 0.35;








cfg.plan.risk_tau = 2.0;














cfg.plan.hybrid.enable      = false;
cfg.plan.hybrid.grid_res    = 0.60;
cfg.plan.hybrid.slice_dt    = 0.60;
cfg.plan.hybrid.yaw_bins    = 24;
cfg.plan.hybrid.step        = 1.20;
cfg.plan.hybrid.steer_set   = [-0.50 -0.28 -0.12 0 0.12 0.28 0.50];
cfg.plan.hybrid.max_expand  = 1400;
cfg.plan.hybrid.max_entry_speed = 2.5;
cfg.plan.hybrid.goal_tol_xy = 2.5;
cfg.plan.hybrid.lookahead   = 11.0;
cfg.plan.hybrid.max_time    = 9.0;
cfg.plan.hybrid.speed       = 1.60;
cfg.plan.hybrid.min_weight  = 0.08;
cfg.plan.hybrid.margin_scale = 0.80;
cfg.plan.hybrid.accept_clear = -0.12;




cfg.plan.hybrid.w_steer     = 0.30;
cfg.plan.hybrid.w_change    = 1.20;
cfg.plan.hybrid.w_offset    = 0.20;
cfg.plan.hybrid.w_heur      = 1.35;


cfg.dec.ttc_caution    = 4.0;
cfg.dec.ttc_emergency  = 1.8;
cfg.dec.clear_caution  = 1.6;
cfg.dec.clear_stop     = 0.7;
cfg.dec.standoff       = 5.0;
cfg.dec.pullout_speed  = 4.0;

cfg.dec.reverse_after    = 3.0;
cfg.dec.reverse_clear    = 1.5;
cfg.dec.reverse_dist     = 3.0;
cfg.dec.reverse_speed    = 0.8;
cfg.dec.reverse_steer    = 0.35;
cfg.dec.reverse_angle    = 0.20;
cfg.dec.reverse_cooldown = 6.0;
cfg.dec.caution_decel  = 1.5;
cfg.dec.caution_margin = 2.0;
cfg.dec.creep_speed    = 1.8;
cfg.dec.caution_scale  = 0.55;
cfg.dec.yield_gap      = 3.5;
cfg.dec.goal_tol       = 3.0;
cfg.dec.arrive_v       = 0.3;
cfg.dec.arrive_decel   = 0.8;
cfg.dec.arrive_lead    = 2.5;
cfg.dec.arrive_tol     = 0.3;




cfg.dec.stuck_cycles     = 18;
cfg.dec.stuck_clear      = 1.0;
cfg.dec.stuck_risk_scale = 1.8;







cfg.track.v_still        = 0.5;
cfg.track.t_still        = 1.0;
cfg.track.v_moving       = 1.0;
cfg.track.v_unstill      = 0.7;
cfg.track.t_move         = 0.5;
cfg.track.two_face_width = 0.5;
cfg.track.extent_decay   = 0.98;
cfg.track.theta_gain     = 0.3;
cfg.track.theta_still_gain = 0.3;
cfg.track.shape_settled  = 3;
cfg.track.ghost_weight   = 0;
cfg.track.establish_hits = 12;
cfg.track.lidar_recent   = 10;
cfg.track.lidar_trust    = 0.85;
cfg.track.piece_frac     = 0.5;
cfg.track.class_size_cap = 1.6;
cfg.track.max_discs      = 12;


cfg.metric.latency_budget = 100.0;
cfg.metric.jerk_limit     = 4.0;
cfg.metric.curv_limit     = 0.30;
cfg.metric.min_clearance  = 0.30;







cfg.debug.think          = false;
cfg.debug.max_candidates = 60;
cfg.debug.cand_points    = 9;


cfg.verbose = true;
end
