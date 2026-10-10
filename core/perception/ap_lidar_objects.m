function [dets, info] = ap_lidar_objects(wx, wy, z, zone, ego, ctx, cfg)










dets = ap_dets_empty();
info = struct('clusters', 0, 'objects', 0);
pc = cfg.percep;
if numel(wx) < pc.min_points
    return;
end
c = cos(ego.psi);
s = sin(ego.psi);


nj = max(1, round(pc.join / pc.cell));
gx = floor((wx - ctx.x0) / pc.cell);
gy = floor((wy - ctx.y0) / pc.cell) + nj + 1;
K = max(gy) + nj + 2;
[keys, ~, cell_of] = unique(gx * K + gy);
nc = numel(keys);
cx = floor(keys / K);
cy = keys - cx * K;



I = zeros(0, 1);
J = zeros(0, 1);
for ox = 0:nj
    for oy = -nj:nj
        if ox == 0 && oy <= 0, continue; end
        if hypot(ox, oy) > nj + 0.5, continue; end
        [tf, loc] = ismember((cx + ox) * K + (cy + oy), keys);
        I = [I; find(tf)];
        J = [J; loc(tf)];
    end
end
lab = local_components(nc, I, J);
[~, ~, lab] = unique(lab);
obj = lab(cell_of);
no = max(obj);
info.clusters = no;









n   = accumarray(obj, 1, [no 1]);
mx  = accumarray(obj, wx, [no 1]) ./ n;
my  = accumarray(obj, wy, [no 1]) ./ n;
dx  = wx - mx(obj);
dy  = wy - my(obj);
na  = pc.fit_angles;
ang = (0:na-1) * (pi / 2) / na;
U   = dx * cos(ang) + dy * sin(ang);
V   = -dx * sin(ang) + dy * cos(ang);
sub = [repmat(obj, na, 1), kron((1:na).', ones(numel(obj), 1))];
U0  = accumarray(sub, U(:), [no na], @min);  U1 = accumarray(sub, U(:), [no na], @max);
V0  = accumarray(sub, V(:), [no na], @min);  V1 = accumarray(sub, V(:), [no na], @max);
oi  = repmat(obj, 1, na);
ai  = repmat(1:na, numel(obj), 1);
li  = oi + (ai - 1) * no;
du  = min(U - U0(li), U1(li) - U);
dv  = min(V - V0(li), V1(li) - V);
close_ = 1 ./ max(min(du, dv), pc.fit_d0);
score = accumarray(sub, close_(:), [no na]);



if isfield(ctx, 'rp')
    rpsi = zeros(no, 1);
    for i = 1:no
        [~, ~, ~, rpsi(i)] = ap_cart2frenet(ctx.rp, mx(i), my(i));
    end
    off = abs(mod(ang - mod(rpsi, pi / 2) + pi / 4, pi / 2) - pi / 4);
    score = score .* (1 + pc.fit_road_bias * (off < deg2rad(1) + (pi / 2) / na));
end
[~, best] = max(score, [], 2);
bl  = (1:no).' + (best - 1) * no;
a   = ang(best).';
u0  = U0(bl); u1 = U1(bl); v0 = V0(bl); v1 = V1(bl);
ocx = mx + cos(a) .* (u0 + u1) / 2 - sin(a) .* (v0 + v1) / 2;
ocy = my + sin(a) .* (u0 + u1) / 2 + cos(a) .* (v0 + v1) / 2;
len = u1 - u0;
wid = v1 - v0;
swap = wid > len;
th  = a + swap * pi / 2;
t_  = len(swap); len(swap) = wid(swap); wid(swap) = t_;
zt  = accumarray(obj, z, [no 1], @max);
zn  = accumarray(obj, zone, [no 1], @max);


ok = n >= pc.min_points & len <= pc.max_extent;
small = zt <= pc.wide_max_height & max(len, wid) <= pc.wide_max_extent;
ok = ok & (zn == 2 | small);
idx = find(ok);
info.objects = numel(idx);
if isempty(idx)
    return;
end




sx = ego.x + c * cfg.ego.rear_axle_to_centre;
sy = ego.y + s * cfg.ego.rear_axle_to_centre;
m = numel(idx);
dets.x = ocx(idx);
dets.y = ocy(idx);
dets.R = zeros(2, 2, m);
dets.box = [len(idx), wid(idx), th(idx)];
dets.box_full = false(m, 1);
dets.origin = repmat([sx, sy], m, 1);
dets.road_psi = NaN(m, 1);
sp = cfg.sensor.lidar.sigma_pos;
for i = 1:m

    sl = sqrt(sp^2 + (len(idx(i)) / 8)^2);
    sw = sqrt(sp^2 + (wid(idx(i)) / 8)^2);
    ct = cos(th(idx(i))); st = sin(th(idx(i)));
    Rot = [ct, -st; st, ct];
    dets.R(:,:,i) = Rot * diag([sl^2, sw^2]) * Rot.';
    if isfield(ctx, 'rp')
        [~, ~, ~, dets.road_psi(i)] = ap_cart2frenet(ctx.rp, dets.x(i), dets.y(i));
    end
end
dets.class = repmat({''}, m, 1);
dets.sensor = repmat({'lidar'}, m, 1);
dets.truth_id = zeros(m, 1);
end


function lab = local_components(n, I, J)


lab = (1:n).';
if isempty(I)
    return;
end
while true
    m = min(lab(I), lab(J));
    nl = min(lab, accumarray([I; J], [m; m], [n 1], @min, n + 1));
    nl = nl(nl);
    if isequal(nl, lab)
        return;
    end
    lab = nl;
end
end
