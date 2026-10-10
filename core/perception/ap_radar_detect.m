function dets = ap_radar_detect(Rr, ego, ctx, cfg)




















dets = ap_dets_empty();
if isempty(Rr)
    return;
end
Rr = double(Rr);
sp = cfg.sensor.radar;
sx = ego.x + cos(ego.psi) * cfg.ego.rear_axle_to_centre;
sy = ego.y + sin(ego.psi) * cfg.ego.rear_axle_to_centre;
b = ego.psi + Rr(:,2);
x = sx + Rr(:,1) .* cos(b);
y = sy + Rr(:,1) .* sin(b);
ground_speed = Rr(:,3) + ego.v * cos(Rr(:,2));
still = abs(ground_speed) < cfg.percep.radar_static_speed;
keep = Rr(:,1) <= sp.range & ap_roi(ctx, x, y) == 2 & ~(still & ap_on_map(ctx, x, y, true));
x = x(keep); y = y(keep); b = b(keep); r = Rr(keep, 1);
m = numel(x);
dets.x = x;
dets.y = y;
dets.R = zeros(2, 2, m);
for i = 1:m
    c = cos(b(i)); s = sin(b(i));
    Rot = [c, -s; s, c];
    dets.R(:,:,i) = Rot * diag([sp.sigma_range^2, (r(i) * sp.sigma_bear)^2]) * Rot.';
end
dets.surface = true(m, 1);
dets.road_psi = local_road_psi(ctx, x, y);
dets.origin = repmat([sx, sy], m, 1);
dets.class = repmat({''}, m, 1);
dets.sensor = repmat({'radar'}, m, 1);
dets.truth_id = zeros(m, 1);
end


function r = local_road_psi(ctx, x, y)

r = NaN(numel(x), 1);
if ~isfield(ctx, 'rp'), return; end
for i = 1:numel(x)
    [~, ~, ~, r(i)] = ap_cart2frenet(ctx.rp, x(i), y(i));
end
end
