function ra = ap_risk_assess(ego, tracks, rp, cfg)



































ra.ttc        = Inf;
ra.lead_id    = 0;
ra.lead_gap   = Inf;
ra.lead_speed = 0;
ra.lead_still = false;
ra.lead_box   = [];
ra.still_ahead = zeros(0, 3);
ra.min_clear  = Inf;
ra.min_clear_moving = Inf;
ra.min_clear_still  = Inf;
ra.cross_ttc  = Inf;
ra.cross_id   = 0;
ra.n_near     = 0;
ra.free_left  = cfg.plan.corridor_halfwidth;
ra.free_right = cfg.plan.corridor_halfwidth;
ra.in_path    = zeros(1, 0);
ra.caution_gap = Inf;
ra.caution_id  = 0;

if isempty(tracks)
    return;
end

[ecx, ecy, erad] = ap_ego_discs(ego.x, ego.y, ego.psi, cfg);


[s_e, d_e] = ap_cart2frenet(rp, ego.x, ego.y);
nose = cfg.ego.length - 0.5 * (cfg.ego.length - cfg.ego.wheelbase);





PATH_HALF = 0.5 * cfg.ego.width + 0.35;

for i = 1:numel(tracks)
    t = tracks(i);
    if strcmp(t.status, 'tentative')
        continue;
    end
    fp = ap_footprint(t, cfg);
    established = ~isfield(t, 'established') || t.established;

    if established && hypot(fp.cx - ego.x, fp.cy - ego.y) < 25
        ra.n_near = ra.n_near + 1;
    end


    if established
        dd = Inf;
        for a = 1:numel(ecx)
            dd = min(dd, min(sqrt((ecx(a) - fp.ox).^2 + (ecy(a) - fp.oy).^2)));
        end
        ra.min_clear = min(ra.min_clear, dd - erad - fp.r);
        if ~fp.still
            ra.min_clear_moving = min(ra.min_clear_moving, dd - erad - fp.r);
        else
            ra.min_clear_still = min(ra.min_clear_still, dd - erad - fp.r);
        end
    end


    sc = zeros(4, 1); dc = zeros(4, 1);
    for k = 1:4
        [sc(k), dc(k)] = ap_cart2frenet(rp, fp.corners(k, 1), fp.corners(k, 2));
    end
    [~, ~, ~, psi_ref] = ap_cart2frenet(rp, fp.cx, fp.cy);
    ahead = min(sc) - s_e;
    lo = min(dc) - d_e;
    hi = max(dc) - d_e;


    v_s =  cos(psi_ref) * fp.vx + sin(psi_ref) * fp.vy;
    v_d = -sin(psi_ref) * fp.vx + cos(psi_ref) * fp.vy;

    in_path = hi > -PATH_HALF && lo < PATH_HALF;



    if established && fp.still && ahead > -1 && ahead < nose + 20
        ra.still_ahead(end+1, :) = [min(sc), min(dc), max(dc)];
    end


    if ~established
        if in_path && max(sc) > s_e
            gap = max(ahead - nose, 0);
            if gap < ra.caution_gap
                ra.caution_gap = gap;
                ra.caution_id  = t.id;
            end
        end
        continue;
    end


    if in_path && max(sc) > s_e
        ra.in_path(end+1) = t.id;
        gap = max(ahead - nose, 0);
        closing = ego.v - v_s;
        if closing > 0.1
            ttc = gap / closing;
            if ttc < ra.ttc
                ra.ttc        = ttc;
                ra.lead_id    = t.id;
                ra.lead_gap   = gap;
                ra.lead_speed = v_s;
                ra.lead_still = fp.still;
                ra.lead_box = [min(sc), min(dc), max(dc)];
            end
        elseif gap < ra.lead_gap


            ra.lead_id    = t.id;
            ra.lead_gap   = gap;
            ra.lead_speed = v_s;
            ra.lead_still = fp.still;
            ra.lead_box = [min(sc), min(dc), max(dc)];
        end
    end





    if ~in_path && ahead > 0 && ahead < 45


        approaching = (lo > 0 && v_d < -0.5) || (hi < 0 && v_d > 0.5);
        if approaching
            lateral_gap = min(abs(lo), abs(hi)) - PATH_HALF;
            t_to_path = max(lateral_gap, 0) / max(abs(v_d), 0.1);





            t_ego = ahead / max(ego.v, 2.0);


            horizon = cfg.pred.horizon;
            if t_to_path < min(t_ego + 2.0, horizon) && t_to_path < ra.cross_ttc
                ra.cross_ttc = t_to_path;
                ra.cross_id  = t.id;
            end
        end
    end





    if max(sc) > s_e - 2 && min(sc) < s_e + 20
        if lo + hi >= 0
            ra.free_left  = min(ra.free_left,  max(lo - cfg.ego.width / 2, 0));
        else
            ra.free_right = min(ra.free_right, max(-hi - cfg.ego.width / 2, 0));
        end
    end
end
end
