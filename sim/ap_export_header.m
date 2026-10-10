function data = ap_export_header(scn, cfg, classes, think)










data.meta.name       = scn.name;
data.meta.desc       = scn.desc;
data.meta.dt         = cfg.sim.dt;
data.meta.replan_dt  = cfg.sim.replan_dt;
data.meta.t_end      = cfg.sim.t_end;
data.meta.goal       = scn.goal;
data.meta.v_max      = cfg.plan.v_max;
data.meta.ego_length = cfg.ego.length;
data.meta.ego_width  = cfg.ego.width;
data.meta.latency_budget = cfg.metric.latency_budget;
data.meta.rear_axle_to_centre = cfg.ego.rear_axle_to_centre;
data.meta.wheelbase  = cfg.ego.wheelbase;

if isfield(scn, 'cross_streets')
    data.meta.cross_streets = reshape(scn.cross_streets.', 1, []);
else
    data.meta.cross_streets = zeros(1, 0);
end



classes = unique(classes);
data.meta.class_size = struct();
for k = 1:numel(classes)
    pr = ap_agent_props(classes{k});
    data.meta.class_size.(classes{k}) = [pr.length, pr.width];
end

data.meta.think = double(think);
if think
    data.meta.sensors.camera = local_sensor(cfg.sensor.camera);
    data.meta.sensors.radar  = local_sensor(cfg.sensor.radar);
    data.meta.sensors.lidar  = local_sensor(cfg.sensor.lidar);
    data.meta.sensors.lidar.beams        = cfg.sensor.lidar.beams;
    data.meta.sensors.lidar.azimuth_bins = cfg.sensor.lidar.azimuth_bins;
    data.meta.sensors.lidar.v_fov        = cfg.sensor.lidar.v_fov;
    data.meta.sensors.lidar.mount_height = cfg.sensor.lidar.mount_height;
    data.meta.pred_mode_names = {'keep', 'brake', 'left', 'right'};
    data.meta.cand_codes = {'ok', 'risk', 'conflict', 'slope', 'curvature', ...
                            'speed', 'lateral_accel'};
end




step = max(1, round(1.0 / scn.rp.ds));
idx  = 1:step:numel(scn.rp.s);
data.road.x  = scn.rp.x(idx).';
data.road.y  = scn.rp.y(idx).';
data.road.hw = scn.rp.halfwidth(idx).';
data.road.psi = scn.rp.psi(idx).';
end


function o = local_sensor(sp)
o.range = sp.range;
o.fov   = sp.fov;
o.pd    = sp.pd;
end
