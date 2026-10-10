function [traj, info] = ap_lattice_plan(ego, rp, pred, bp, cfg)









































t_start = tic;


[s0, d0, k_ref, psi_ref] = ap_cart2frenet(rp, ego.x, ego.y);

dpsi   = ap_wrap_pi(ego.psi - psi_ref);
one_kd = max(1 - k_ref * d0, 0.1);

s0_dot  = ego.v * cos(dpsi) / one_kd;
d0_dot  = ego.v * sin(dpsi);
















s0_ddot = 0;
d0_ddot = 0;


halfwidth = interp1(rp.s, rp.halfwidth, min(max(s0, 0), rp.length), 'linear');
d_limit   = min(bp.d_max, halfwidth);






n_off = max(3, round(2 * d_limit / cfg.plan.d_step) + 1);
d_set = linspace(-d_limit, d_limit, n_off);



















d_set = unique([d_set, d0]);






d_lane = ap_lane_centre(halfwidth, cfg);
lane_w = 1;
if isfield(bp, 'lane_w'), lane_w = bp.lane_w; end
if abs(d_lane) <= d_limit
    d_set = unique([d_set, d_lane]);
end












v_cap = max(bp.v_cap, 0);

T_max = max(cfg.plan.horizon_T);
reach = (2/3) * T_max;
v_hi  = min(v_cap, ego.v + reach * cfg.veh.a_max);
v_lo  = max(0, ego.v + reach * cfg.veh.a_min);
v_lo  = min(v_lo, v_hi);

v_set = unique(max(linspace(v_lo, v_hi, cfg.plan.v_samples), 0));

nD = numel(d_set);
nV = numel(v_set);

best      = [];
best_cost = Inf;








relax       = [];
relax_score = Inf;

n_eval = 0;
n_feas = 0;






n_rej = struct('lon', 0, 'slope', 0, 'curv', 0, 'speed', 0, 'alat', 0, 'risk', 0, 'road', 0, 'hold', 0);





think = isfield(cfg, 'debug') && isfield(cfg.debug, 'think') && cfg.debug.think;
if think
    cand = struct('x', zeros(cfg.debug.cand_points, 0), ...
                  'y', zeros(cfg.debug.cand_points, 0), ...
                  'cost', zeros(1, 0), 'risk', zeros(1, 0), 'code', zeros(1, 0));
end

for T = cfg.plan.horizon_T
    tv = (0:cfg.pred.dt:T)';
    nT = numel(tv);
    if nT < 3
        continue;
    end





    z  = zeros(nT, 1);
    o  = ones(nT, 1);
    P0 = [o, tv, tv.^2, tv.^3,   tv.^4,    tv.^5];
    P1 = [z, o,  2*tv,  3*tv.^2, 4*tv.^3,  5*tv.^4];
    P2 = [z, z,  2*o,   6*tv,    12*tv.^2, 20*tv.^3];
    P3 = [z, z,  z,     6*o,     24*tv,    60*tv.^2];

    LonC = zeros(nV, 5);
    for i = 1:nV
        LonC(i,:) = ap_quartic(s0, s0_dot, s0_ddot, v_set(i), 0, T);
    end

    S   = P0(:,1:5) * LonC.';
    Sd  = P1(:,1:5) * LonC.';
    Sdd = P2(:,1:5) * LonC.';
    Sj  = P3(:,1:5) * LonC.';




    lon_ok = all(Sd  >= -0.05, 1) & ...
             all(Sdd <=  cfg.veh.a_max * 1.05, 1) & ...
             all(Sdd >=  cfg.veh.a_min * 1.05, 1);

    vi_ok  = find(lon_ok);
    n_eval = n_eval + nD * nV;
    n_rej.lon = n_rej.lon + nD * sum(~lon_ok);
    if isempty(vi_ok)
        continue;
    end


    nP = nD * numel(vi_ok);
    DI = repmat(1:nD, 1, numel(vi_ok));
    VI = reshape(repmat(vi_ok(:).', nD, 1), 1, nP);

    Sp  = S(:, VI);
    Sdp = Sd(:, VI);
    Sjp = Sj(:, VI);
















    DS    = max(Sp - s0, 0);
    Strav = DS(end, :);


    dprime0 = d0_dot / max(s0_dot, 0.5);

    LatC = zeros(6, nP);
    for q = 1:nP
        d1q = d_set(DI(q));
        if Strav(q) < 0.5



            LatC(:, q) = [d0; 0; 0; 0; 0; 0];
        else
            LatC(:, q) = ap_quintic(d0, dprime0, 0, d1q, 0, 0, Strav(q)).';
        end
    end


    Dp  = zeros(nT, nP);
    DPr = zeros(nT, nP);
    for k = 0:5
        Ck = ones(nT, 1) * LatC(k+1, :);
        Dp = Dp + Ck .* DS.^k;
        if k >= 1
            DPr = DPr + k * Ck .* DS.^(k-1);
        end
    end


    Ddp = DPr .* Sdp;
    Djp = ap_gradient_cols(ap_gradient_cols( ...
              ap_gradient_cols(Dp, cfg.pred.dt), cfg.pred.dt), cfg.pred.dt);


















    holo_ok = all(abs(DPr) <= cfg.plan.max_path_slope, 1);








    hw_s = interp1(rp.s, rp.halfwidth, min(max(Sp, 0), rp.length), 'linear');   % [nT x nP]
    road_lim = max(hw_s - 0.5 * cfg.ego.width + cfg.plan.road_overhang, abs(d0) + 1e-3);
    road_ok = all(abs(Dp) <= road_lim, 1);
    holo_ok = holo_ok & road_ok;
    n_rej.road = n_rej.road + sum(~road_ok);


    [cxv, cyv, cpv] = ap_frenet2cart(rp, Sp(:), Dp(:), DPr(:));
    CX   = reshape(cxv, nT, nP);
    CY   = reshape(cyv, nT, nP);
    CPSI = reshape(cpv, nT, nP);


    V = sqrt((Sdp .* one_kd).^2 + Ddp.^2);




    dx  = ap_gradient_cols(CX, cfg.pred.dt);
    dy  = ap_gradient_cols(CY, cfg.pred.dt);
    ddx = ap_gradient_cols(dx, cfg.pred.dt);
    ddy = ap_gradient_cols(dy, cfg.pred.dt);

    v2  = dx.^2 + dy.^2;
    den = v2.^1.5;
    den(den < 1e-9) = 1e-9;
    KAP = (dx .* ddy - dy .* ddx) ./ den;








    KAP(v2 < 0.30^2) = 0;

    max_k  = max(abs(KAP), [], 1);
    max_v  = max(V, [], 1);
    max_al = max(V.^2 .* abs(KAP), [], 1);

    ok_curv  = max_k  <= cfg.metric.curv_limit * 1.5;







    ok_speed = max_v  <= max(v_cap, ego.v) + 1.5;
    ok_alat  = max_al <= cfg.veh.a_lat_max * 1.4;

    n_rej.slope = n_rej.slope + sum(~holo_ok);
    n_rej.curv  = n_rej.curv  + sum(holo_ok & ~ok_curv);
    n_rej.speed = n_rej.speed + sum(holo_ok & ok_curv & ~ok_speed);
    n_rej.alat  = n_rej.alat  + sum(holo_ok & ok_curv & ok_speed & ~ok_alat);





    hold_ok = true(1, nP);
    if isfield(bp, 'hold') && ~isempty(bp.hold)
        nose = cfg.ego.length - 0.5 * (cfg.ego.length - cfg.ego.wheelbase);
        band = 0.5 * cfg.ego.width + 0.3;
        for h = 1:size(bp.hold, 1)
            ends_close = Sp(end, :) + nose > bp.hold(h, 1) & Sp(end, :) < bp.hold(h, 1) + cfg.dec.standoff & Strav > 0.05;
            in_line = Dp(end, :) > bp.hold(h, 2) - band & Dp(end, :) < bp.hold(h, 3) + band;
            hold_ok = hold_ok & ~(ends_close & in_line);
        end
    end
    n_rej.hold = n_rej.hold + sum(holo_ok & ~hold_ok);

    kin_ok = holo_ok & ok_curv & ok_speed & ok_alat & hold_ok;

    if think
        code = zeros(1, nP);
        code(~ok_alat)  = 6;
        code(~ok_speed) = 5;
        code(~ok_curv)  = 4;
        code(~holo_ok)  = 3;
        rej = find(~kin_ok);
        ks  = round(linspace(1, nT, cfg.debug.cand_points));
        cand.x    = [cand.x,    CX(ks, rej)];
        cand.y    = [cand.y,    CY(ks, rej)];
        cand.cost = [cand.cost, NaN(1, numel(rej))];
        cand.risk = [cand.risk, NaN(1, numel(rej))];
        cand.code = [cand.code, code(rej)];
    end


    keep = find(kin_ok);
    if isempty(keep)
        continue;
    end
    [COLL, MINCLR, RISK] = ap_collision_check( ...
        CX(:,keep), CY(:,keep), CPSI(:,keep), tv, pred, cfg);

    for q = 1:numel(keep)
        p         = keep(q);
        collides  = COLL(q);
        min_clear = MINCLR(q);
        risk      = RISK(q);

        d1 = d_set(DI(p));
        v1 = v_set(VI(p));

        cost = local_cost(Sp(:,p), Sdp(:,p), Sjp(:,p), Djp(:,p), d1 - d_lane, v1, T, ...
                          max_k(p), min_clear, risk, s0, v_cap, cfg, lane_w);






        score = risk - 0.01 * min(min_clear, 5);
        if score < relax_score
            relax_score = score;
            relax       = local_pack(tv, CX, CY, CPSI, V, Sp, Dp, KAP, p, ...
                                     cost, risk, min_clear, T, d1, v1, cfg, false);
        end

        if think
            ccode = 0;
            if risk > bp.risk_tol
                ccode = 1;
            elseif collides && min_clear < -0.05 && risk > 0.5 * bp.risk_tol
                ccode = 2;
            end
            cand.x    = [cand.x,    CX(ks, p)];
            cand.y    = [cand.y,    CY(ks, p)];
            cand.cost = [cand.cost, cost];
            cand.risk = [cand.risk, risk];
            cand.code = [cand.code, ccode];
        end

        if risk > bp.risk_tol
            n_rej.risk = n_rej.risk + 1;
            continue;
        end


        if collides && min_clear < -0.05 && risk > 0.5 * bp.risk_tol
            continue;
        end

        n_feas = n_feas + 1;
        if cost < best_cost
            best_cost = cost;
            best      = local_pack(tv, CX, CY, CPSI, V, Sp, Dp, KAP, p, ...
                                   cost, risk, min_clear, T, d1, v1, cfg, true);
        end
    end
end






info.hybrid_used     = false;
info.hybrid_expanded = 0;
info.stagnant        = false;














stagnant = ~isempty(best) && best.v_target < 0.30 && ego.v < 0.50 && ...
           bp.v_cap > 0.30;

if isempty(best) || stagnant
    info.stagnant = stagnant;












    may_search = cfg.plan.hybrid.enable && ...
                 ego.v < cfg.plan.hybrid.max_entry_speed && ...
                 (isempty(best) || (isfield(bp, 'unstick') && bp.unstick));

    if may_search
        [hyb, hinfo] = ap_hybrid_astar(ego, rp, pred, bp, cfg);
        info.hybrid_expanded = hinfo.expanded;










        if ~isempty(hyb) && hyb.min_clear > cfg.plan.hybrid.accept_clear && ...
           (hyb.s(end) - hyb.s(1)) > 1.0
            best = hyb;
            info.hybrid_used = true;
        end
    end

    if isempty(best)
        if ~isempty(relax) && relax.min_clear > 0


            best = relax;
        else
            best = local_emergency_stop(ego, rp, s0, d0, cfg);
        end
    end
end

traj = best;

info.feasible    = traj.feasible;
info.n_evaluated = n_eval;
info.n_feasible  = n_feas;
info.n_rejected  = n_rej;
info.latency_ms  = toc(t_start) * 1000;
info.s0          = s0;
info.d0          = d0;
info.d_limit     = d_limit;

if think
    info.cand = local_thin_candidates(cand, cfg.debug.max_candidates);
end
end


function c = local_thin_candidates(c, cap)



n = numel(c.code);
if n <= cap
    return;
end
acc = find(c.code == 0);
[~, o] = sort(c.cost(acc));
acc = acc(o);
n_acc = min(numel(acc), round(0.6 * cap));
keep = acc(1:n_acc);
rej = find(c.code ~= 0);
n_rej = min(numel(rej), cap - n_acc);
if n_rej > 0
    keep = [keep, rej(unique(round(linspace(1, numel(rej), n_rej))))];
end
c.x = c.x(:, keep);  c.y = c.y(:, keep);
c.cost = c.cost(keep);  c.risk = c.risk(keep);  c.code = c.code(keep);
end


function c = local_pack(tv, CX, CY, CPSI, V, Sp, Dp, KAP, p, ...
                        cost, risk, min_clear, T, d1, v1, cfg, feasible)



c = struct('t', tv, 'x', CX(:,p), 'y', CY(:,p), 'psi', CPSI(:,p), ...
           'v', V(:,p), 'a', ap_gradient(V(:,p), cfg.pred.dt), ...
           's', Sp(:,p), 'd', Dp(:,p), 'kappa', KAP(:,p), ...
           'cost', cost, 'risk', risk, 'min_clear', min_clear, ...
           'T', T, 'd_target', d1, 'v_target', v1, ...
           'emergency', false, 'feasible', feasible);
end


function J = local_cost(s_t, sd_t, js_t, jd_t, d1, v1, T, max_k, ...
                        min_clear, risk, s0, v_cap, cfg, lane_w)






J = cfg.plan.w_risk * risk;



J = J + cfg.plan.w_jerk * (ap_mean(js_t.^2) + ap_mean(jd_t.^2)) / 100;


J = J + cfg.plan.w_curv * max_k^2;


shortfall = max(0, (s0 + v_cap * T) - s_t(end)) / max(T, 0.1);
J = J + cfg.plan.w_progress * shortfall;




J = J + lane_w * cfg.plan.w_offset * d1^2;


J = J + cfg.plan.w_speed_dev * (v1 - v_cap)^2;





CLEAR_REF = 1.5;
if isfinite(min_clear)
    J = J + cfg.plan.w_clearance * max(0, CLEAR_REF - min_clear)^2;
end



if ap_mean(sd_t) < 0.3
    J = J + 25;
end
end


function traj = local_emergency_stop(ego, rp, s0, d0, cfg)




tv = (0:cfg.pred.dt:max(cfg.plan.horizon_T))';

a  = cfg.veh.a_emergency;
v  = max(ego.v + a * tv, 0);
ds = ego.v * tv + 0.5 * a * tv.^2;



t_stop = -ego.v / a;
ds(tv > t_stop) = ego.v * t_stop + 0.5 * a * t_stop^2;
ds = max(ds, 0);

s_t = s0 + ds;
d_t = d0 * ones(size(tv));

[cx, cy, cpsi] = ap_frenet2cart(rp, s_t, d_t, zeros(size(tv)));

traj = struct('t', tv, 'x', cx, 'y', cy, 'psi', cpsi, 'v', v, ...
              'a', a * ones(size(tv)), ...
              's', s_t, 'd', d_t, 'kappa', zeros(size(tv)), ...
              'cost', Inf, 'risk', 1, 'min_clear', -Inf, ...
              'T', tv(end), 'd_target', d0, 'v_target', 0, ...
              'emergency', true, 'feasible', false);
end
