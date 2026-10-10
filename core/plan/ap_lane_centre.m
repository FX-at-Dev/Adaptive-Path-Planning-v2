function d = ap_lane_centre(halfwidth, cfg)







d = 0;
if isfield(cfg.plan, 'keep_left') && cfg.plan.keep_left && 2 * halfwidth >= cfg.plan.two_lane_width
    d = halfwidth / 2;
end
end
