function [tracks, next_id, diag] = ap_tracker_step(tracks, dets, dt, next_id, cfg)


























if isempty(tracks)
    tracks = local_empty_track_array();
end

nd = numel(dets.x);


for i = 1:numel(tracks)
    [tracks(i).x, tracks(i).P] = ap_ekf_predict(tracks(i).x, tracks(i).P, dt, cfg);
    tracks(i).age   = tracks(i).age + 1;
    tracks(i).z_age = tracks(i).z_age + 1;
    tracks(i).seen  = false;
    tracks(i).lidar_age = tracks(i).lidar_age + 1;
end








dets = local_merge_pieces(tracks, dets, cfg);
nd = numel(dets.x);


sensor_names = {'camera', 'radar', 'lidar'};
leftover = false(nd, 1);

for si = 1:numel(sensor_names)
    idx = find(strcmp(dets.sensor, sensor_names{si}));
    if isempty(idx)
        continue;
    end
    [tracks, unassigned] = local_associate(tracks, dets, idx, dt, cfg);
    leftover(unassigned) = true;
end











leftover = leftover & ~local_fragment(tracks, dets, cfg);
groups = local_cluster(dets, find(leftover));
for g = 1:numel(groups)
    tracks(end+1) = local_birth(next_id, dets, groups{g}, cfg);
    next_id = next_id + 1;
end



lidar_at = [];
if isfield(dets, 'origin') && ~isempty(dets.origin)
    k = find(strcmp(dets.sensor(:), 'lidar') & all(isfinite(dets.origin), 2), 1);
    if ~isempty(k), lidar_at = dets.origin(k, :); end
end
keep = true(1, numel(tracks));
for i = 1:numel(tracks)
    t = tracks(i);






    if t.x(3) < cfg.track.v_still
        t.slow_t = t.slow_t + dt;
    elseif ~t.still
        t.slow_t = 0;
    end



    if t.x(3) >= cfg.track.v_unstill
        t.fast_t = t.fast_t + dt;
    else
        t.fast_t = 0;
    end
    if t.still && t.fast_t >= cfg.track.t_move
        t.still = false;
        t.slow_t = 0;
    elseif ~t.still
        t.still = t.slow_t >= cfg.track.t_still;
    end
    t.still = t.still || strcmp(t.class, 'static');










    in_reach = ~isempty(lidar_at) && hypot(t.x(1) - lidar_at(1), t.x(2) - lidar_at(2)) <= ...
               cfg.track.lidar_trust * cfg.sensor.lidar.range;
    t.established = t.established || (t.hits >= cfg.track.establish_hits && ...
                    (~in_reach || t.lidar_age <= cfg.track.lidar_recent));

    if t.seen
        t.hits   = t.hits + 1;
        t.misses = 0;
        t.hist   = [t.hist, true];
    else
        t.misses = t.misses + 1;
        t.hist   = [t.hist, false];
    end








    recent = t.hist(max(1, end - cfg.fuse.confirm_N + 1):end);
    if strcmp(t.status, 'tentative') && sum(recent) >= cfg.fuse.confirm_M && local_corroborated(t, lidar_at, cfg)
        t.status = 'confirmed';
    end

    if t.misses > 0 && strcmp(t.status, 'confirmed')
        t.status = 'coasting';
    elseif t.misses == 0 && strcmp(t.status, 'coasting')
        t.status = 'confirmed';
    end




    if strcmp(t.status, 'tentative') && (t.misses >= 2 || t.age > cfg.fuse.tentative_max_age)
        keep(i) = false;
    elseif t.misses >= cfg.fuse.max_misses && ~(t.still && t.misses < cfg.fuse.max_misses_still)



        keep(i) = false;
    end

    tracks(i) = t;
end
tracks = tracks(keep);







tracks = local_prune_duplicates(tracks, cfg);


diag.n_det       = nd;
diag.n_track     = numel(tracks);
diag.n_confirmed = 0;
for i = 1:numel(tracks)
    if ~strcmp(tracks(i).status, 'tentative')
        diag.n_confirmed = diag.n_confirmed + 1;
    end
end
diag.n_new = numel(groups);
end


function dets = local_merge_pieces(tracks, dets, cfg)

n = numel(dets.x);
if n < 2 || isempty(tracks) || ~isfield(dets, 'box')
    return;
end
isl = strcmp(dets.sensor(:), 'lidar') & all(isfinite(dets.box), 2) & ~dets.box_full(:);
if sum(isl) < 2
    return;
end
used = false(n, 1);
drop = false(n, 1);
for i = 1:numel(tracks)
    t = tracks(i);



    if strcmp(t.status, 'tentative') || ...
            ((t.shape_n < cfg.track.shape_settled || ~all(isfinite(t.shape))) && isempty(t.cls_names))
        continue;
    end
    fp = ap_footprint(t, cfg);
    u = [cos(fp.theta); sin(fp.theta)];
    v = [-u(2); u(1)];
    du = (dets.x(:) - fp.cx) * u(1) + (dets.y(:) - fp.cy) * u(2);
    dv = (dets.x(:) - fp.cx) * v(1) + (dets.y(:) - fp.cy) * v(2);
    cand = find(isl & ~used & abs(du) <= fp.L / 2 + 0.3 & abs(dv) <= fp.W / 2 + 0.3);
    if numel(cand) < 2
        continue;
    end

    pu = zeros(4 * numel(cand), 1);
    pv = pu;
    for q = 1:numel(cand)
        j = cand(q);
        c = cos(dets.box(j, 3)); s = sin(dets.box(j, 3));
        hl = dets.box(j, 1) / 2; hw = dets.box(j, 2) / 2;
        cx = dets.x(j) + c * [hl, hl, -hl, -hl] - s * [hw, -hw, -hw, hw];
        cy = dets.y(j) + s * [hl, hl, -hl, -hl] + c * [hw, -hw, -hw, hw];
        pu(4*q-3:4*q) = (cx - fp.cx) * u(1) + (cy - fp.cy) * u(2);
        pv(4*q-3:4*q) = (cx - fp.cx) * v(1) + (cy - fp.cy) * v(2);
    end
    u0 = min(pu); u1 = max(pu); v0 = min(pv); v1 = max(pv);



    if u1 - u0 > fp.L + 0.5 || v1 - v0 > fp.W + 0.5
        continue;
    end



    um = (u0 + u1) / 2; vm = (v0 + v1) / 2;
    u0 = max(u0, um - fp.L / 2); u1 = min(u1, um + fp.L / 2);
    v0 = max(v0, vm - fp.W / 2); v1 = min(v1, vm + fp.W / 2);
    k = cand(1);
    ctr = [fp.cx; fp.cy] + u * (u0 + u1) / 2 + v * (v0 + v1) / 2;
    dets.x(k) = ctr(1);
    dets.y(k) = ctr(2);
    if u1 - u0 >= v1 - v0
        dets.box(k, :) = [u1 - u0, v1 - v0, fp.theta];
    else
        dets.box(k, :) = [v1 - v0, u1 - u0, ap_wrap_pi(fp.theta + pi / 2)];
    end
    sp = cfg.sensor.lidar.sigma_pos;
    sl = sqrt(sp^2 + (dets.box(k, 1) / 8)^2);
    sw = sqrt(sp^2 + (dets.box(k, 2) / 8)^2);
    Rot = [cos(dets.box(k, 3)), -sin(dets.box(k, 3)); sin(dets.box(k, 3)), cos(dets.box(k, 3))];
    dets.R(:, :, k) = Rot * diag([sl^2, sw^2]) * Rot.';
    used(cand) = true;
    drop(cand(2:end)) = true;
end
if any(drop)
    keep = ~drop;
    f = fieldnames(dets);
    for q = 1:numel(f)
        val = dets.(f{q});
        if strcmp(f{q}, 'R')
            dets.R = val(:, :, keep);
        elseif size(val, 1) == n
            dets.(f{q}) = val(keep, :);
        end
    end
end
end


function in = local_fragment(tracks, dets, cfg)

in = false(numel(dets.x), 1);
for i = 1:numel(tracks)
    t = tracks(i);
    if strcmp(t.status, 'tentative')
        continue;
    end
    fp = ap_footprint(t, cfg);
    u = [cos(fp.theta), sin(fp.theta)];
    dx = dets.x - fp.cx;
    dy = dets.y - fp.cy;
    along = abs(u(1) * dx + u(2) * dy);
    across = abs(-u(2) * dx + u(1) * dy);
    in = in | (strcmp(dets.sensor, 'lidar') & along <= fp.L / 2 & across <= fp.W / 2);
end
end


function ok = local_corroborated(t, lidar_at, cfg)




ok = t.src(3) > 0;
if ~ok && t.src(1) > 0 && t.src(2) > 0
    ok = isempty(lidar_at) || ...
         hypot(t.x(1) - lidar_at(1), t.x(2) - lidar_at(2)) > cfg.track.lidar_trust * cfg.sensor.lidar.range;
end
end


function k = local_sensor_index(name)
switch name
    case 'camera', k = 1;
    case 'radar',  k = 2;
    otherwise,     k = 3;
end
end


function in = local_inside(a, b, cfg)

in = false;
if strcmp(b.status, 'tentative') || ~isfield(b, 'shape') || ~all(isfinite(b.shape))
    return;
end
fp = ap_footprint(b, cfg);
u = [cos(fp.theta); sin(fp.theta)];
d = [a.x(1) - fp.cx; a.x(2) - fp.cy];
in = abs(u.' * d) <= fp.L / 2 + 0.3 && abs([-u(2), u(1)] * d) <= fp.W / 2 + 0.3;
end


function on = local_on_footprint(t, dets, j, cfg)

on = false;
if strcmp(t.status, 'tentative') || ~isfield(dets, 'box') || size(dets.box, 1) < j || ...
        any(~isfinite(dets.box(j, :))) || ~isfield(t, 'shape') || ~all(isfinite(t.shape))
    return;
end
fp = ap_footprint(t, cfg);
u = [cos(fp.theta); sin(fp.theta)];
d = [dets.x(j) - fp.cx; dets.y(j) - fp.cy];


on = abs(u.' * d) <= fp.L / 2 + 0.3 && abs([-u(2), u(1)] * d) <= fp.W / 2 + 0.3;
end


function tracks = local_prune_duplicates(tracks, cfg)





POS_TOL   = 3.5;
SPD_TOL   = 4.0;
HEAD_TOL  = 0.7;
POS_HARD  = 1.2;

n = numel(tracks);
if n < 2
    return;
end

drop = false(1, n);
for i = 1:n
    if drop(i) || strcmp(tracks(i).status, 'tentative')
        continue;
    end
    for j = (i+1):n
        if drop(j) || strcmp(tracks(j).status, 'tentative')
            continue;
        end
        d = hypot(tracks(i).x(1) - tracks(j).x(1), ...
                  tracks(i).x(2) - tracks(j).x(2));
        if d > POS_TOL && d > 7
            continue;
        end

        same = (d < POS_HARD);




        if ~same && (local_inside(tracks(i), tracks(j), cfg) || local_inside(tracks(j), tracks(i), cfg))
            same = abs(tracks(i).x(3) - tracks(j).x(3)) < SPD_TOL;
        end



        if ~same && d <= POS_TOL && xor(tracks(i).src(3) > 0, tracks(j).src(3) > 0)
            slow = tracks(i).x(3) < cfg.track.v_moving && tracks(j).x(3) < cfg.track.v_moving;
            dh = abs(ap_wrap_pi(tracks(i).x(4) - tracks(j).x(4)));
            same = abs(tracks(i).x(3) - tracks(j).x(3)) < SPD_TOL && (slow || dh < HEAD_TOL);
            if same

                if tracks(i).src(3) > 0, drop(j) = true; else, drop(i) = true; break; end
                continue;
            end
        end
        if ~same && d <= POS_TOL
            dv = abs(tracks(i).x(3) - tracks(j).x(3));
            dh = abs(ap_wrap_pi(tracks(i).x(4) - tracks(j).x(4)));
            same = (dv < SPD_TOL) && (dh < HEAD_TOL);
        end
        if ~same
            continue;
        end



        if tracks(i).hits > tracks(j).hits || ...
          (tracks(i).hits == tracks(j).hits && tracks(i).age >= tracks(j).age)
            drop(j) = true;
        else
            drop(i) = true;
            break;
        end
    end
end
tracks = tracks(~drop);
end


function [tracks, unassigned] = local_associate(tracks, dets, idx, dt, cfg)

BIG = 1e6;

nt = numel(tracks);
nj = numel(idx);
unassigned = idx;

if nt == 0 || nj == 0
    return;
end

C = BIG * ones(nt, nj);





Lt = zeros(nt, 1); St = zeros(nt, 1); Xt = zeros(nt, 2);
for i = 1:nt
    Lt(i) = ap_agent_props(tracks(i).class).length;
    if all(isfinite(tracks(i).shape)), Lt(i) = max(Lt(i), tracks(i).shape(1)); end
    St(i) = max(tracks(i).P(1,1), 0) + max(tracks(i).P(2,2), 0);
    Xt(i, :) = tracks(i).x(1:2).';
end
Lb = zeros(1, nj);
if isfield(dets, 'box') && size(dets.box, 1) >= max(idx)
    Lb = dets.box(idx, 1).';
    Lb(~isfinite(Lb)) = 0;
end
Sd = reshape(dets.R(1,1,idx) + dets.R(2,2,idx), 1, nj);
reach = (Lt + Lb) / 2 + 3.1 * sqrt(St + Sd) + 1.0;
near = hypot(Xt(:, 1) - dets.x(idx).', Xt(:, 2) - dets.y(idx).') <= reach;
for i = 1:nt
    for jj = find(near(i, :))
        j = idx(jj);


        [~, zc, ~, Rc] = ap_track_shape(tracks(i), dets, j, cfg);
        [~, ~, nis, valid] = ap_ekf_update(tracks(i).x, tracks(i).P, ...
            zc, Rc, cfg.fuse.gate_chi2);
        if valid
            C(i, jj) = nis;
        elseif local_on_footprint(tracks(i), dets, j, cfg)



            C(i, jj) = 0.9 * cfg.fuse.gate_chi2;
        end
    end
end

assign = ap_hungarian(C);
took   = false(nj, 1);

for i = 1:nt
    jj = assign(i);


    if jj > 0 && C(i, jj) < BIG / 2
        tracks(i) = local_apply_update(tracks(i), dets, idx(jj), dt, cfg);
        took(jj)  = true;
    end
end

unassigned = idx(~took);
end


function t = local_apply_update(t, dets, j, dt, cfg)
t = local_vote(t, dets.class{j});




[t, z, shift, Rz] = ap_track_shape(t, dets, j, cfg);
t.x(1:2) = t.x(1:2) + shift;







if t.hits == 1 && ~t.seen && t.z_age >= 1
    elapsed = t.z_age * dt;
    vx = (z(1) - t.last_z(1)) / elapsed;
    vy = (z(2) - t.last_z(2)) / elapsed;
    sp = hypot(vx, vy);
    if sp > 0.3
        t.x(3)   = min(sp, 25);
        t.x(4)   = atan2(vy, vx);
        t.P(3,3) = 4.0;
        t.P(4,4) = 0.5;
    end
end

[t.x, t.P] = ap_ekf_update(t.x, t.P, z, Rz, []);








if t.x(3) < 0
    t.x(3) = -t.x(3);
    t.x(4) = ap_wrap_pi(t.x(4) + pi);
    J = diag([1, 1, -1, 1, 1]);
    t.P = J * t.P * J.';
end

t.seen   = true;
t.last_z = z;
t.z_age  = 0;
si = local_sensor_index(dets.sensor{j});
t.src(si) = t.src(si) + 1;
if si == 3, t.lidar_age = 0; end
end


function t = local_vote(t, cls)


if ~isempty(cls)
    k = find(strcmp(t.cls_names, cls), 1);
    if isempty(k)
        t.cls_names{end+1}  = cls;
        t.cls_counts(end+1) = 1;
    else
        t.cls_counts(k) = t.cls_counts(k) + 1;
    end
    [~, best] = max(t.cls_counts);
    t.class = t.cls_names{best};
end
end


function groups = local_cluster(dets, idx)




groups = {};
if isempty(idx)
    return;
end

RADIUS = 2.0;
used   = false(numel(idx), 1);

for a = 1:numel(idx)
    if used(a)
        continue;
    end
    g = idx(a);
    used(a) = true;
    for b = (a+1):numel(idx)
        if used(b)
            continue;
        end
        if hypot(dets.x(idx(b)) - dets.x(idx(a)), ...
                 dets.y(idx(b)) - dets.y(idx(a))) < RADIUS
            g = [g; idx(b)];
            used(b) = true;
        end
    end
    groups{end+1} = g;
end
end


function t = local_birth(id, dets, group, cfg)



cls  = '';
for k = 1:numel(group)
    if isempty(cls) && ~isempty(dets.class{group(k)})
        cls = dets.class{group(k)};
    end
end

t.id = id;
t.x = zeros(5, 1);
t.P = eye(5);

if isempty(cls)
    t.class      = 'car';        % neutral prior until the camera weighs in
    t.cls_names  = {};
    t.cls_counts = [];
else
    t.class      = cls;
    t.cls_names  = {cls};
    t.cls_counts = 1;
end
t.shape    = [NaN, NaN, NaN];
t.shape_n  = 0;
t.still    = false;
t.slow_t   = 0;
t.fast_t   = 0;
t.road_psi = NaN;
t.src      = zeros(1, 3);
t.lidar_age = 1e6;
t.established = false;
for k = 1:numel(group)
    si = local_sensor_index(dets.sensor{group(k)});
    t.src(si) = t.src(si) + 1;
    if si == 3, t.lidar_age = 0; end
end



Info = zeros(2, 2);
Ivec = zeros(2, 1);
for k = 1:numel(group)
    j    = group(k);
    [t, zj] = ap_track_shape(t, dets, j, cfg);
    Ri   = inv(dets.R(:,:,j));
    Info = Info + Ri;
    Ivec = Ivec + Ri * zj;
end
P0 = inv(Info);
z0 = P0 * Ivec;



t.x = [z0(1); z0(2); 0; 0; 0];
t.P = diag([max(P0(1,1), 1.0), max(P0(2,2), 1.0), 9.0, 1.0, 0.5]);

t.hits   = 1;
t.misses = 0;
t.age    = 1;
t.status = 'tentative';
t.hist   = true;
t.last_z = z0;
t.z_age  = 0;
t.seen   = true;
end


function t = local_empty_track_array()
t = struct('id', {}, 'x', {}, 'P', {}, 'class', {}, 'cls_names', {}, ...
           'cls_counts', {}, 'shape', {}, 'shape_n', {}, 'still', {}, ...
           'slow_t', {}, 'fast_t', {}, 'road_psi', {}, 'src', {}, 'lidar_age', {}, 'established', {}, 'hits', {}, 'misses', {}, 'age', {}, ...
           'status', {}, 'hist', {}, 'last_z', {}, 'z_age', {}, 'seen', {});
end
