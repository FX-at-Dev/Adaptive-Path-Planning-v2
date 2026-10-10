function [collides, min_clear, risk] = ap_collision_check(ex, ey, epsi, te, pred, cfg)





































nT = size(ex, 1);
nC = size(ex, 2);

collides  = false(1, nC);
min_clear = Inf(1, nC);
risk      = zeros(1, nC);

if isempty(pred) || ~isfield(pred, 'x') || isempty(pred.x)
    return;
end

A = size(pred.x, 2);
if A == 0
    return;
end

K  = numel(pred.t);
nd = cfg.ego.n_discs;


[dcx, dcy, rego] = ap_ego_discs(ex(:), ey(:), epsi(:), cfg);
margin = rego + cfg.plan.safety_margin;



EGX = zeros(nT, nC * nd);
EGY = zeros(nT, nC * nd);
for i = 1:nd
    EGX(:, (i-1)*nC + (1:nC)) = reshape(dcx(:, i), nT, nC);
    EGY(:, (i-1)*nC + (1:nC)) = reshape(dcy(:, i), nT, nC);
end




ki = round(te(:) / cfg.pred.dt) + 1;
ki = min(max(ki, 1), K);

AX = pred.x(ki, :);
AY = pred.y(ki, :);

ones_cols = ones(1, nC * nd);






if isfield(cfg.plan, 'risk_tau') && cfg.plan.risk_tau > 0
    disc = exp(-te(:) / cfg.plan.risk_tau);
else
    disc = ones(nT, 1);
end
sev = zeros(A, nC);

for a = 1:A
    dx = EGX - AX(:, a) * ones_cols;
    dy = EGY - AY(:, a) * ones_cols;
    d2 = dx.^2 + dy.^2;

    thresh  = margin + pred.r(a);
    thresh2 = thresh^2;


    mind2 = min(d2, [], 1);
    clr   = sqrt(mind2) - thresh;



    clr_c = min(reshape(clr, nC, nd), [], 2).';
    min_clear = min(min_clear, clr_c);















    escaping = (mind2 >= 0.98 * d2(1, :)) & (d2(end, :) > 1.02 * d2(1, :));
    [hasv, firstrow] = max((d2 < thresh2) & ~(ones(nT, 1) * escaping), [], 1);
    sev_dc = double(hasv) .* disc(firstrow).';
    sev(a, :) = max(reshape(sev_dc, nC, nd), [], 2).';
end

if ~any(sev(:) > 0)
    return;
end

collides = any(sev > 0, 1);




if isfield(pred, 'hyp') && ~isempty(pred.hyp)
    [hyp_ids, ia] = unique(pred.hyp(:).');
    w_hyp = pred.w(ia);

    nH      = numel(hyp_ids);
    hyp_sev = zeros(nH, nC);
    for h = 1:nH
        cols = (pred.hyp(:).' == hyp_ids(h));
        hyp_sev(h, :) = max(sev(cols, :), [], 1);
    end

    risk = w_hyp(:).' * hyp_sev;
else
    risk = pred.w(:).' * sev;
end




risk = min(risk, 1);
end
