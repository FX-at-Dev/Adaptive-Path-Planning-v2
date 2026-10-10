function grid = ap_occupancy_grid(ego, pred, cfg)























res    = cfg.plan.hybrid.grid_res;
ahead  = cfg.risk.ahead;
behind = cfg.risk.behind;
lat    = cfg.risk.lateral;



R  = max(ahead, max(behind, lat)) + 6;
x0 = ego.x - R;
y0 = ego.y - R;
n  = ceil(2*R / res) + 1;


nt = max(1, ceil(cfg.pred.horizon / cfg.plan.hybrid.slice_dt));
dt = cfg.plan.hybrid.slice_dt;

grid.occ = false(n, n, nt);
grid.x0  = x0;
grid.y0  = y0;
grid.res = res;
grid.dt  = dt;
grid.nt  = nt;
grid.n   = n;

if isempty(pred) || ~isfield(pred, 'x') || isempty(pred.x)
    return;
end








[~, ~, rego] = ap_ego_discs(ego.x, ego.y, ego.psi, cfg);
ego_infl = rego + cfg.plan.hybrid.margin_scale * cfg.plan.safety_margin;

K = numel(pred.t);
A = size(pred.x, 2);

for it = 1:nt

    t_mid = (it - 0.5) * dt;
    ki = min(max(round(t_mid / cfg.pred.dt) + 1, 1), K);

    for a = 1:A



        if pred.w(a) < cfg.plan.hybrid.min_weight
            continue;
        end

        cx = pred.x(ki, a);
        cy = pred.y(ki, a);
        rr = pred.r(a) + ego_infl;

        ix_lo = floor((cx - rr - x0) / res) + 1;
        ix_hi = ceil( (cx + rr - x0) / res) + 1;
        iy_lo = floor((cy - rr - y0) / res) + 1;
        iy_hi = ceil( (cy + rr - y0) / res) + 1;

        ix_lo = max(ix_lo, 1);  ix_hi = min(ix_hi, n);
        iy_lo = max(iy_lo, 1);  iy_hi = min(iy_hi, n);
        if ix_lo > ix_hi || iy_lo > iy_hi
            continue;
        end

        gx = x0 + ((ix_lo:ix_hi) - 1) * res;
        gy = y0 + ((iy_lo:iy_hi) - 1) * res;

        dx = gx(:) * ones(1, numel(gy)) - cx;
        dy = ones(numel(gx), 1) * gy - cy;

        mask = (dx.^2 + dy.^2) <= rr^2;
        if any(mask(:))
            block = grid.occ(ix_lo:ix_hi, iy_lo:iy_hi, it);
            grid.occ(ix_lo:ix_hi, iy_lo:iy_hi, it) = block | mask;
        end
    end
end
end
