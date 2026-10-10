function f = ap_export_frame(s)








f.t     = s.t;
f.state = s.state;
f.ego   = [s.ego.x, s.ego.y, s.ego.psi, s.ego.v];

na = numel(s.agents);
f.agents = zeros(na, 3);
f.agent_class = cell(1, na);
f.agent_id = zeros(1, na);
for i = 1:na
    f.agents(i,:)    = [s.agents(i).x, s.agents(i).y, s.agents(i).psi];
    f.agent_class{i} = s.agents(i).class;
    f.agent_id(i)    = s.agents(i).id;
end

nt = numel(s.tracks);
f.tracks = zeros(nt, 4);
f.track_class = cell(1, nt);
for i = 1:nt
    f.tracks(i,:)    = [s.tracks(i).x, s.tracks(i).y, s.tracks(i).psi, s.tracks(i).v];
    f.track_class{i} = s.tracks(i).class;
end


f.traj = [s.traj_x(:).'; s.traj_y(:).'];

if isfield(s, 'think')
    f.think = local_think(s.think);
end
end


function o = local_think(th)












nt = numel(th.tracks);
T = zeros(nt, 14);
for i = 1:nt
    tr = th.tracks(i);
    T(i,:) = [tr.id, tr.x, tr.y, tr.psi, tr.v, tr.P, ...
              find(strcmp(tr.status, {'tentative', 'confirmed', 'coasting', 'unconfirmed'})) - 1, ...
              tr.L, tr.W, tr.theta, tr.still, tr.in_path];
end
o.tracks = local_flat(T, [0 2 2 3 2 4 4 4 0 2 2 3 0 0]);
o.track_class = {th.tracks.class};
o.track_stride = 14;



nd = numel(th.dets.x);
D = zeros(nd, 6);
for i = 1:nd
    b = th.dets.box(i, :);
    b(~isfinite(b)) = -1;
    D(i,:) = [th.dets.x(i), th.dets.y(i), ...
              find(strcmp(th.dets.sensor{i}, {'camera', 'radar', 'lidar'})) - 1, b];
end
o.dets = local_flat(D, [2 2 0 2 2 3]);
o.det_stride = 6;



np = numel(th.pred);
P = zeros(np, 4);
xy = zeros(np, 0);
for i = 1:np
    h = th.pred(i);
    ks = 1:2:numel(h.x);
    P(i,:) = [h.src, find(strcmp(h.mode, {'keep', 'brake', 'left', 'right'})) - 1, h.w, h.r];
    xy(i, 1:2*numel(ks)) = reshape([h.x(ks); h.y(ks)], 1, []);
end
o.pred = local_flat(P, [0 0 3 2]);
o.pred_stride = 4;
o.pred_xy = local_flat(xy, 2);
o.pred_pts = size(xy, 2) / 2;




c = th.cand;
nc = numel(c.code);
C = [c.code(:), c.cost(:), c.risk(:)];
C(isnan(C)) = -1;
o.cand = local_flat(C, [0 2 3]);
o.cand_stride = 3;
XY = zeros(nc, 2 * size(c.x, 1));
XY(:, 1:2:end) = c.x.';
XY(:, 2:2:end) = c.y.';
o.cand_xy = local_flat(XY, 2);
o.cand_pts = size(c.x, 1);


ra = th.ra;
o.ra = round([local_fin(ra.ttc), ra.lead_id, local_fin(ra.lead_gap), ...
              ra.lead_speed, local_fin(ra.min_clear), local_fin(ra.cross_ttc), ...
              ra.cross_id, ra.n_near, ra.free_left, ra.free_right] * 100) / 100;
o.bp = round([th.bp.v_cap, th.bp.d_max, th.bp.risk_tol] * 1000) / 1000;
o.reason = th.bp.reason;
in = th.info;
r = in.n_rejected;
o.info = [in.n_evaluated, in.n_feasible, r.lon, r.slope, r.curv, r.speed, ...
          r.alat, r.risk, round(in.d_limit * 100) / 100, ...
          round(in.latency_ms * 10) / 10, double(in.feasible)];
o.look = round(th.look * 100) / 100;
end


function v = local_flat(M, digits)

if isempty(M)
    v = zeros(1, 0);
    return;
end
q = 10 .^ digits;
if isscalar(q)
    q = repmat(q, 1, size(M, 2));
end
M = round(M .* q) ./ q;
v = reshape(M.', 1, []);
end


function v = local_fin(v)

if ~isfinite(v)
    v = 999;
end
end
