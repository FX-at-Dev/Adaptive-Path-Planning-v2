function pred = ap_predict_intent(tracks, cfg)

































pred.t   = (0:cfg.pred.dt:cfg.pred.horizon)';
K        = numel(pred.t);

pred.x   = zeros(K, 0);
pred.y   = zeros(K, 0);
pred.r   = zeros(1, 0);
pred.w   = zeros(1, 0);
pred.hyp = zeros(1, 0);
pred.src = zeros(1, 0);
pred.mode = zeros(1, 0);
pred.mode_names = {'keep', 'brake', 'left', 'right'};

if isempty(tracks)
    return;
end

hyp_id = 0;

for i = 1:numel(tracks)
    t = tracks(i);



    if strcmp(t.status, 'tentative')
        continue;
    end

    props = ap_agent_props(t.class);
    fp = ap_footprint(t, cfg);




    sigma0 = sqrt(max(t.P(1,1), 0) + max(t.P(2,2), 0));

    [modes, weights] = local_hypotheses(props);
    if fp.still
        modes = {'keep'};
        weights = 1;
    end





    if isfield(t, 'established') && ~t.established
        weights = weights * cfg.track.ghost_weight;
        if cfg.track.ghost_weight <= 0, continue; end
    end

    for m = 1:numel(modes)
        if weights(m) < 0.02
            continue;
        end
        hyp_id = hyp_id + 1;

        if fp.still
            hx = fp.cx * ones(K, 1);
            hy = fp.cy * ones(K, 1);
            hpsi = fp.theta * ones(K, 1);
        else
            x0 = t.x;
            x0(1) = fp.cx; x0(2) = fp.cy;
            [hx, hy, hpsi] = local_rollout(x0, modes{m}, props, pred.t, cfg);
        end











        spd = abs(t.x(3)) * ~fp.still;
        mob = max(0.2, min(1.0, spd / max(props.v_typ, 0.5)));
        grow = cfg.pred.sigma_grow * (0.5 + props.erratic) * mob * pred.t;
        rad  = fp.r + cfg.risk.inflate + 0.5 * sigma0;




        nd = fp.n;
        offs = -fp.L / 2 + (fp.L / nd) * ((1:nd) - 0.5);
        cx = hx * ones(1, nd) + cos(hpsi) * offs;
        cy = hy * ones(1, nd) + sin(hpsi) * offs;







        rcol = rad + ap_mean(grow);

        pred.x   = [pred.x,   cx];
        pred.y   = [pred.y,   cy];
        pred.r   = [pred.r,   rcol * ones(1, nd)];
        pred.w   = [pred.w,   weights(m) * ones(1, nd)];
        pred.hyp = [pred.hyp, hyp_id * ones(1, nd)];
        pred.src = [pred.src, t.id * ones(1, nd)];
        pred.mode = [pred.mode, m * ones(1, nd)];
    end
end
end


function [modes, w] = local_hypotheses(props)












e = props.erratic;
y = props.yields;

w_keep  = 0.88 - 0.50 * e;
w_brake = 0.10 + 0.18 * y;
w_side  = max(0, 1 - w_keep - w_brake);

modes = {'keep', 'brake', 'left', 'right'};
w     = [w_keep, w_brake, w_side/2, w_side/2];
w     = w / sum(w);
end


function [hx, hy, hpsi] = local_rollout(x0, mode, props, tvec, cfg)

K   = numel(tvec);
dt  = cfg.pred.dt;

hx   = zeros(K, 1);
hy   = zeros(K, 1);
hpsi = zeros(K, 1);

x = x0(:);




v_lat = 0;
switch mode
    case 'left'
        v_lat =  props.lat_agility * min(1, 0.3 + abs(x(3)) / max(props.v_typ, 1));
    case 'right'
        v_lat = -props.lat_agility * min(1, 0.3 + abs(x(3)) / max(props.v_typ, 1));
end

a_lon = 0;
if strcmp(mode, 'brake')
    a_lon = -0.6 * props.a_brake;
end



LAT_CAP    = 2.5;
lat_travel = 0;

hx(1)   = x(1);
hy(1)   = x(2);
hpsi(1) = x(4);

dt2 = dt^2;
for k = 2:K



    v = x(3); psi = x(4); w = x(5);
    if abs(w) > 1e-4
        psi1 = psi + w * dt;
        x(1) = x(1) + (v / w) * (sin(psi1) - sin(psi));
        x(2) = x(2) + (v / w) * (cos(psi) - cos(psi1));
        x(4) = psi1;
    else
        x(1) = x(1) + v * cos(psi) * dt - 0.5 * v * sin(psi) * w * dt2;
        x(2) = x(2) + v * sin(psi) * dt + 0.5 * v * cos(psi) * w * dt2;
        x(4) = psi + w * dt;
    end


    x(3) = max(0, x(3) + a_lon * dt);












    if v_lat ~= 0 && abs(lat_travel) < LAT_CAP
        step = v_lat * dt;
        x(1) = x(1) - step * sin(x(4));
        x(2) = x(2) + step * cos(x(4));
        lat_travel = lat_travel + step;
    end

    hx(k)   = x(1);
    hy(k)   = x(2);
    hpsi(k) = x(4);
end
end
