function [dets, info] = ap_lidar_detect(P, ego, ctx, cfg)




























dets = ap_dets_empty();
info = struct('points', size(P, 1), 'kept', 0, 'clusters', 0, 'objects', 0, 'kept_pts', zeros(0, 4, 'single'));
if isempty(P)
    return;
end
p = double(P);
pc = cfg.percep;


c = cos(ego.psi);
s = sin(ego.psi);
bx = p(:,1) + cfg.ego.rear_axle_to_centre;
by = p(:,2);
z  = p(:,3) + cfg.sensor.lidar.mount_height;
wx = ego.x + c .* bx - s .* by;
wy = ego.y + s .* bx + c .* by;
keep = z > pc.ground_height & z < pc.max_height;


zone = zeros(size(wx));
zone(keep) = ap_roi(ctx, wx(keep), wy(keep));
keep = keep & zone > 0;
keep(keep) = ~ap_on_map(ctx, wx(keep), wy(keep));
wx = wx(keep); wy = wy(keep); z = z(keep); zone = zone(keep);
info.kept = numel(wx);
info.kept_pts = single([wx, wy, z, zone]);


[dets, o] = ap_lidar_objects(wx, wy, z, zone, ego, ctx, cfg);
info.clusters = o.clusters;
info.objects = o.objects;
end

