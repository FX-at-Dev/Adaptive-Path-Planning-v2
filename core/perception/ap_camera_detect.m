function dets = ap_camera_detect(B, ego, ctx, cfg)




























dets = ap_dets_empty();
if isempty(B)
    return;
end
B = double(B);
sp = cfg.sensor.camera;
W = sp.image_size(1); H = sp.image_size(2);
f = (W / 2) / tan(sp.fov / 2);
cxp = W / 2; cyp = H / 2;
h = sp.mount_height;
names = ap_class_names();
heights = [1.5 3.2 3.0 1.85 1.55 1.65 1.7 1.45 1.3 1.3];

um = (B(:,1) + B(:,3)) / 2;
left = (cxp - um) / f;
down = (B(:,4) - cyp) / f;
ci = min(max(round(B(:,5)), 1), numel(names));



fwd_ground = h ./ max(down, 1e-6);
fwd_height = heights(ci).' * f ./ max(B(:,4) - B(:,2), 1);
edge = cfg.percep.camera_edge_px;
cut_bottom = B(:,4) >= H - edge;
cut_top = B(:,2) <= edge;
cut_side = B(:,1) <= edge | B(:,3) >= W - edge;
valid = ~cut_side & ~(cut_bottom & cut_top);
use_ground = down > tan(deg2rad(1.0)) & ~cut_bottom;
fwd = fwd_height;
fwd(use_ground) = fwd_ground(use_ground);
lat = fwd .* left;
rng = hypot(fwd, lat);







c = cos(ego.psi); s = sin(ego.psi);
sx = ego.x + c * cfg.ego.rear_axle_to_centre;
sy = ego.y + s * cfg.ego.rear_axle_to_centre;
x = sx + c * fwd - s * lat;
y = sy + s * fwd + c * lat;
keep = valid & rng <= sp.range & ap_roi(ctx, x, y) > 0;
idx = find(keep);
m = numel(idx);
dets.x = x(idx);
dets.y = y(idx);
dets.R = zeros(2, 2, m);
for i = 1:m
    k = idx(i);
    b = atan2(y(k) - sy, x(k) - sx);
    sr = sp.sigma_range * rng(k);
    Rot = [cos(b), -sin(b); sin(b), cos(b)];
    dets.R(:,:,i) = Rot * diag([sr^2, (rng(k) * sp.sigma_bear)^2]) * Rot.';
end
dets.surface = true(m, 1);
dets.origin = repmat([sx, sy], m, 1);
dets.road_psi = NaN(m, 1);
if isfield(ctx, 'rp')
    for i = 1:m
        [~, ~, ~, dets.road_psi(i)] = ap_cart2frenet(ctx.rp, dets.x(i), dets.y(i));
    end
end
dets.class = reshape(names(ci(idx)), [], 1);
dets.sensor = repmat({'camera'}, m, 1);
dets.truth_id = zeros(m, 1);
end
