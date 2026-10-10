function ctx = ap_perception_init(rp, cfg, static_map)



















ctx.rp = rp;
c = cfg.percep.roi_cell;
hw = rp.halfwidth(:);
pad = max(hw) + cfg.percep.roi_wide + 1;
ctx.cell = c;
ctx.x0 = min(rp.x) - pad;
ctx.y0 = min(rp.y) - pad;
ctx.nx = ceil((max(rp.x) + pad - ctx.x0) / c) + 1;
ctx.ny = ceil((max(rp.y) + pad - ctx.y0) / c) + 1;
roi = zeros(ctx.ny, ctx.nx, 'uint8');



step = max(1, round(1.0 / rp.ds));
idx = [1:step:numel(rp.x), numel(rp.x)];
for i = idx
    for zone = 1:2
        if zone == 1
            r = hw(i) + cfg.percep.roi_wide;
        else
            r = hw(i) + cfg.percep.roi_near;
        end
        gx0 = max(1, floor((rp.x(i) - r - ctx.x0) / c) + 1);
        gx1 = min(ctx.nx, ceil((rp.x(i) + r - ctx.x0) / c) + 1);
        gy0 = max(1, floor((rp.y(i) - r - ctx.y0) / c) + 1);
        gy1 = min(ctx.ny, ceil((rp.y(i) + r - ctx.y0) / c) + 1);
        [GX, GY] = meshgrid(gx0:gx1, gy0:gy1);
        px = ctx.x0 + (GX - 0.5) * c;
        py = ctx.y0 + (GY - 0.5) * c;
        in = (px - rp.x(i)).^2 + (py - rp.y(i)).^2 <= r^2;
        lin = sub2ind([ctx.ny, ctx.nx], GY(in), GX(in));
        roi(lin) = max(roi(lin), uint8(zone));
    end
end
ctx.roi = roi;


mc = cfg.percep.map_cell;
ctx.map_cell = mc;
ctx.mnx = ceil(ctx.nx * c / mc) + 1;
ctx.mny = ceil(ctx.ny * c / mc) + 1;
ctx.static = false(ctx.mny, ctx.mnx);
ctx.static_radar = false(ctx.mny, ctx.mnx);
if nargin < 3 || isempty(static_map)
    return;
end
for i = 1:size(static_map, 1)
    f = static_map(i, :);
    for pass = 1:2
        if pass == 1, g = cfg.percep.map_margin; else, g = cfg.percep.radar_map_margin; end
        hl = f(3) / 2 + g;
        hw2 = f(4) / 2 + g;
        rr = hypot(hl, hw2);
        gx0 = max(1, floor((f(1) - rr - ctx.x0) / mc) + 1);
        gx1 = min(ctx.mnx, ceil((f(1) + rr - ctx.x0) / mc) + 1);
        gy0 = max(1, floor((f(2) - rr - ctx.y0) / mc) + 1);
        gy1 = min(ctx.mny, ceil((f(2) + rr - ctx.y0) / mc) + 1);
        if gx1 < gx0 || gy1 < gy0, continue; end
        [GX, GY] = meshgrid(gx0:gx1, gy0:gy1);
        dx = ctx.x0 + (GX - 0.5) * mc - f(1);
        dy = ctx.y0 + (GY - 0.5) * mc - f(2);
        u = dx * cos(f(5)) + dy * sin(f(5));
        v = -dx * sin(f(5)) + dy * cos(f(5));
        in = abs(u) <= hl & abs(v) <= hw2;
        lin = sub2ind([ctx.mny, ctx.mnx], GY(in), GX(in));
        if pass == 1
            ctx.static(lin) = true;
        else
            ctx.static_radar(lin) = true;
        end
    end
end
end
