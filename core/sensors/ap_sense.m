function dets = ap_sense(world, cfg, use)



























if nargin < 3
    use = true(1, 3);
end

specs = {cfg.sensor.camera, cfg.sensor.radar, cfg.sensor.lidar};
names = {'camera', 'radar', 'lidar'};

dets = ap_dets_empty();


[bx, by, br, bid] = local_blockers(world.agents);

for si = 1:numel(specs)
    if ~use(si)
        continue;
    end
    d = local_scan(world, specs{si}, names{si}, bx, by, br, bid, cfg);
    dets = ap_dets_cat(dets, d);
end
end


function dets = local_scan(world, spec, name, bx, by, br, bid, cfg)


dets = ap_dets_empty();
ego  = world.ego;

for k = 1:numel(world.agents)
    a = world.agents(k);
    if ~a.active
        continue;
    end

    dx = a.x - ego.x;
    dy = a.y - ego.y;
    rng_true = hypot(dx, dy);

    if rng_true > spec.range || rng_true < 0.1
        continue;
    end

    bearing_global = atan2(dy, dx);
    bearing_rel    = ap_wrap_pi(bearing_global - ego.psi);
    if abs(bearing_rel) > spec.fov / 2
        continue;
    end


    if cfg.sensor.occlusion
        own = (bid == a.id);
        if ap_ray_blocked(ego.x, ego.y, a.x, a.y, bx(~own), by(~own), br(~own))
            continue;
        end
    end


    pd = spec.pd * (1 - 0.40 * (rng_true / spec.range)^2);
    if rand() > pd
        continue;
    end


    switch name
        case 'camera'

            sr = spec.sigma_range * rng_true;
            sb = spec.sigma_bear;
        case 'radar'

            sr = spec.sigma_range;
            sb = spec.sigma_bear;
        case 'lidar'
            sr = spec.sigma_pos;
            sb = spec.sigma_pos / max(rng_true, 1.0);
        otherwise
            error('ap_sense:sensor', 'Unknown sensor "%s".', name);
    end

    r_meas = rng_true + sr * randn();
    b_meas = bearing_global + sb * randn();

    zx = ego.x + r_meas * cos(b_meas);
    zy = ego.y + r_meas * sin(b_meas);


    c = cos(bearing_global);
    s = sin(bearing_global);
    Rot = [c, -s; s, c];
    R = Rot * diag([sr^2, (rng_true * sb)^2]) * Rot.';


    if spec.classifies
        if rand() < spec.class_acc
            cls = a.class;
        else
            cls = local_confuse(a.class);
        end
    else
        cls = '';
    end

    dets = local_push(dets, zx, zy, R, cls, name, a.id);

    [~, ~, ~, road_psi] = ap_cart2frenet(world.rp, zx, zy);
    n = numel(dets.x);
    dets.road_psi(n) = road_psi;
    if strcmp(name, 'lidar')

        dets.box(n, :) = [a.props.length, a.props.width, a.psi];
        dets.box_full(n) = true;
        dets.origin(n, :) = [ego.x, ego.y];
    end
end





n_fa = local_poisson(cfg.sensor.clutter_rate);
for k = 1:n_fa
    rr = spec.range * sqrt(rand());
    bb = ego.psi + (rand() - 0.5) * spec.fov;
    zx = ego.x + rr * cos(bb);
    zy = ego.y + rr * sin(bb);
    R  = diag([1.0, 1.0]);
    dets = local_push(dets, zx, zy, R, '', name, 0);
end
end


function [bx, by, br, bid] = local_blockers(agents)

bx = []; by = []; br = []; bid = [];
for k = 1:numel(agents)
    a = agents(k);
    if ~a.active
        continue;
    end
    [cx, cy] = ap_agent_discs(a.x, a.y, a.psi, a.props);
    n = numel(cx);
    bx  = [bx;  cx(:)];
    by  = [by;  cy(:)];
    br  = [br;  a.props.radius * ones(n, 1)];
    bid = [bid; a.id * ones(n, 1)];
end
end


function cls = local_confuse(true_class)





switch true_class
    case 'car',         opts = {'auto', 'truck'};
    case 'auto',        opts = {'car', 'pushcart'};
    case 'bus',         opts = {'truck'};
    case 'truck',       opts = {'bus', 'car'};
    case 'two_wheeler', opts = {'bicycle'};
    case 'bicycle',     opts = {'two_wheeler', 'pedestrian'};
    case 'pedestrian',  opts = {'bicycle', 'cattle'};
    case 'cattle',      opts = {'pedestrian', 'pushcart'};
    case 'pushcart',    opts = {'cattle', 'auto'};
    otherwise,          opts = {'car'};
end
cls = opts{min(numel(opts), 1 + floor(rand() * numel(opts)))};
end


function n = local_poisson(lambda)


if lambda <= 0
    n = 0;
    return;
end
L = exp(-lambda);
k = 0;
p = 1;
while true
    k = k + 1;
    p = p * rand();
    if p <= L
        break;
    end
end
n = k - 1;
end


function d = local_push(d, x, y, R, cls, sensor, tid)
n = numel(d.x) + 1;
d.x(n,1) = x;
d.y(n,1) = y;
d.R(:,:,n) = R;
d.class{n,1} = cls;
d.sensor{n,1} = sensor;
d.truth_id(n,1) = tid;
d.box(n,:) = [NaN, NaN, NaN];
d.box_full(n,1) = false;
d.origin(n,:) = [NaN, NaN];
d.road_psi(n,1) = NaN;
d.surface(n,1) = false;
end
