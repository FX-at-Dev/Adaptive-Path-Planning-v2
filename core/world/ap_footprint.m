function fp = ap_footprint(t, cfg)




























props = ap_agent_props(t.class);
fp.cx = t.x(1);
fp.cy = t.x(2);
fp.still = isfield(t, 'still') && t.still;
has_shape = isfield(t, 'shape') && ~isempty(t.shape) && all(isfinite(t.shape));
has_road = isfield(t, 'road_psi') && ~isempty(t.road_psi) && isfinite(t.road_psi);
moving = ~fp.still && t.x(3) >= cfg.track.v_moving && t.hits >= 3;

settled = has_shape && isfield(t, 'shape_n') && t.shape_n >= cfg.track.shape_settled;
if settled



    fp.theta = t.shape(3);
    if moving && cos(t.x(4) - fp.theta) < 0
        fp.theta = ap_wrap_pi(fp.theta + pi);
    end
elseif moving || ~(has_shape || has_road)
    fp.theta = t.x(4);
elseif has_shape
    fp.theta = t.shape(3);
else
    fp.theta = t.road_psi;
end

if has_shape
    fp.L = max(t.shape(1), 0.3);
    fp.W = max(t.shape(2), 0.3);

    fp.n = max(1, min(cfg.track.max_discs, ceil(fp.L / (2 * sqrt(0.15 * (fp.W + 0.15))))));
    fp.r = sqrt((fp.L / (2 * fp.n))^2 + (fp.W / 2)^2);
else
    fp.L = props.length;
    fp.W = props.width;
    fp.n = props.n_discs;
    fp.r = props.radius;
end

if fp.still
    fp.vx = 0; fp.vy = 0;
else
    fp.vx = t.x(3) * cos(t.x(4));
    fp.vy = t.x(3) * sin(t.x(4));
end

c = cos(fp.theta); s = sin(fp.theta);
offs = -fp.L / 2 + (fp.L / fp.n) * ((1:fp.n) - 0.5);
fp.ox = fp.cx + c * offs;
fp.oy = fp.cy + s * offs;
hl = fp.L / 2; hw = fp.W / 2;
u = [hl, hl, -hl, -hl]; v = [hw, -hw, -hw, hw];
fp.corners = [fp.cx + c * u - s * v; fp.cy + s * u + c * v].';
end
