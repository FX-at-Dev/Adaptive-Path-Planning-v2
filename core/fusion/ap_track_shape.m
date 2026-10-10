function [t, z, shift, Rz] = ap_track_shape(t, dets, j, cfg)







































z = [dets.x(j); dets.y(j)];
shift = [0; 0];
Rz = dets.R(:, :, j);

if isfield(dets, 'road_psi') && numel(dets.road_psi) >= j && isfinite(dets.road_psi(j))
    t.road_psi = dets.road_psi(j);
end
if ~isfield(dets, 'box') || size(dets.box, 1) < j || any(~isfinite(dets.box(j, 1:3)))








    shp = [];
    if t.shape_n > 0 && all(isfinite(t.shape))
        shp = t.shape;
    elseif ~isempty(t.cls_names)
        pr = ap_agent_props(t.class);
        th0 = t.road_psi;
        if t.x(3) >= cfg.track.v_moving && t.hits >= 3, th0 = t.x(4); end
        if isfinite(th0), shp = [pr.length, pr.width, th0]; end
    end
    if isfield(dets, 'surface') && numel(dets.surface) >= j && dets.surface(j) && ...
            ~isempty(shp) && all(isfinite(dets.origin(j, :)))
        los = z - dets.origin(j, :).';
        r = norm(los);
        if r > 0.1
            a = atan2(los(2), los(1)) - shp(3);
            h = abs(shp(1) / 2 * cos(a)) + abs(shp(2) / 2 * sin(a));
            z = z + h * los / r;
        end
    end
    return;
end
Lv = dets.box(j, 1); Wv = dets.box(j, 2); thv = dets.box(j, 3);
full = isfield(dets, 'box_full') && numel(dets.box_full) >= j && dets.box_full(j);

classified = ~isempty(t.cls_names);
prior = [];
if classified
    prior = ap_agent_props(t.class);
end
moving = t.x(3) >= cfg.track.v_moving && t.hits >= 3;















two_faces = full || Wv >= cfg.track.two_face_width;
wmax = 2.6;
if classified, wmax = prior.width + 0.4; end
if full
    along = true;
elseif Lv > wmax
    along = true;
elseif t.shape_n >= cfg.track.shape_settled && all(isfinite(t.shape))
    along = abs(local_axis_diff(thv, t.shape(3))) < pi / 4;
elseif moving
    along = abs(local_axis_diff(thv, t.x(4))) < pi / 4;
elseif isfield(t, 'road_psi') && isfinite(t.road_psi)
    along = abs(local_axis_diff(thv, t.road_psi)) < pi / 4;
elseif classified && ~two_faces
    along = Lv >= 0.7 * prior.length;
else
    along = true;
end
Wseen = Wv * two_faces;
if along
    axis = thv; Lo = Lv; Wo = Wseen; seen_u = Lv; seen_v = Wv;
else
    axis = thv + pi / 2; Lo = Wseen; Wo = Lv; seen_u = Wv; seen_v = Lv;
end






piece_u = false; piece_v = false;
if ~full && t.shape_n >= cfg.track.shape_settled && all(isfinite(t.shape))
    if along
        piece_u = seen_u < cfg.track.piece_frac * t.shape(1);
    else
        piece_v = seen_v < cfg.track.piece_frac * t.shape(2);
    end
end
piece = piece_u || piece_v;


old_shape = t.shape;
if t.shape_n == 0 || any(~isfinite(t.shape))
    th = axis;
    W = max(Wo, 0.3);
    if Wo == 0, W = 0.6; end
    L = max([Lo, W, 0.6]);
else



    g = cfg.track.theta_gain;
    if t.still && ~two_faces
        g = 0;
    elseif t.still
        g = g * cfg.track.theta_still_gain;
    end

    if piece, g = 0; end
    th = t.shape(3) + g * local_axis_diff(axis, t.shape(3));




    d = cfg.track.extent_decay;
    if piece || ~two_faces, d = 1; end
    L = max(Lo, d * t.shape(1));
    W = max(Wo, d * t.shape(2));
end
if classified


    cap = cfg.track.class_size_cap;
    L = min(max(L, prior.length), cap * prior.length);
    W = min(max(W, prior.width), cap * prior.width);
end
t.shape = [L, W, ap_wrap_pi(th)];
t.shape_n = t.shape_n + 1;


if ~full && isfield(dets, 'origin') && size(dets.origin, 1) >= j && all(isfinite(dets.origin(j, :)))
    z_seen = z;
    u = [cos(t.shape(3)); sin(t.shape(3))];
    v = [-u(2); u(1)];
    if piece_u



        z = local_centre(z_seen, t.shape, t.shape(1), seen_v, dets.origin(j, :).');
        Rz = Rz + (max(0, t.shape(1) - seen_u) / 2)^2 * (u * u.');
    elseif piece_v

        z = local_centre(z_seen, t.shape, seen_u, t.shape(2), dets.origin(j, :).');
        Rz = Rz + (max(0, t.shape(2) - seen_v) / 2)^2 * (v * v.');
    else
        z = local_centre(z_seen, t.shape, seen_u, seen_v, dets.origin(j, :).');
        if all(isfinite(old_shape))
            shift = z - local_centre(z_seen, old_shape, seen_u, seen_v, dets.origin(j, :).');
        end
    end
end
end


function z = local_centre(z, shape, seen_u, seen_v, origin)


u = [cos(shape(3)); sin(shape(3))];
v = [-u(2); u(1)];
los = z - origin;
hid_u = max(0, shape(1) - seen_u);
hid_v = max(0, shape(2) - seen_v);
z = z + local_sign(u.' * los) * hid_u / 2 * u + local_sign(v.' * los) * hid_v / 2 * v;
end


function d = local_axis_diff(a, b)

d = mod(a - b + pi / 2, pi) - pi / 2;
end

function s = local_sign(x)
s = 1;
if x < 0, s = -1; end
end
