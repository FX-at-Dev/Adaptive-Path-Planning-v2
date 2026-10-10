function s = ap_snapshot(world, st, tk, t, min_clear)











s.t   = t;
s.ego = world.ego;
s.min_clear = min_clear;
s.state = st.bp.state;

s.agents = struct('x', {}, 'y', {}, 'psi', {}, 'class', {}, 'id', {});
for k = 1:numel(world.agents)
    a = world.agents(k);
    if ~a.active
        continue;
    end
    s.agents(end+1) = struct('x', a.x, 'y', a.y, 'psi', a.psi, ...
                             'class', a.class, 'id', a.id);
end

s.tracks = struct('x', {}, 'y', {}, 'psi', {}, 'v', {}, 'class', {}, 'status', {});
for k = 1:numel(st.tracks)
    tr = st.tracks(k);
    if strcmp(tr.status, 'tentative')
        continue;
    end
    s.tracks(end+1) = struct('x', tr.x(1), 'y', tr.x(2), 'psi', tr.x(4), ...
                             'v', tr.x(3), 'class', tr.class, 'status', tr.status);
end

if isempty(st.traj)
    s.traj_x = [];
    s.traj_y = [];
else
    s.traj_x = st.traj.x;
    s.traj_y = st.traj.y;
end

if st.think
    s.think = local_think(st.tracks, tk.dets, st.pred, st.ra, st.bp, st.info, tk.look, cfg_of(st));
end
end


function th = local_think(tracks, dets, pred, ra, bp, info, look, cfg)



th.tracks = struct('id', {}, 'x', {}, 'y', {}, 'psi', {}, 'v', {}, ...
                   'class', {}, 'status', {}, 'P', {}, 'L', {}, 'W', {}, ...
                   'theta', {}, 'still', {}, 'in_path', {});
inp = zeros(1, 0);
if isfield(ra, 'in_path'), inp = ra.in_path; end
for k = 1:numel(tracks)
    tr = tracks(k);
    fp = ap_footprint(tr, cfg);
    status = tr.status;
    if ~strcmp(status, 'tentative') && isfield(tr, 'established') && ~tr.established
        status = 'unconfirmed';      % slowed for, not yet stopped for
    end
    th.tracks(end+1) = struct('id', tr.id, 'x', fp.cx, 'y', fp.cy, ...
        'psi', tr.x(4), 'v', tr.x(3) * ~fp.still, 'class', tr.class, 'status', status, ...
        'P', [tr.P(1,1), tr.P(1,2), tr.P(2,2)], 'L', fp.L, 'W', fp.W, ...
        'theta', fp.theta, 'still', double(fp.still), 'in_path', double(any(inp == tr.id)));
end

th.dets.x = dets.x(:)';
th.dets.y = dets.y(:)';
th.dets.sensor = dets.sensor(:)';
th.dets.box = NaN(numel(dets.x), 3);
if isfield(dets, 'box') && size(dets.box, 1) == numel(dets.x)
    th.dets.box = dets.box;
end




th.pred = struct('src', {}, 'mode', {}, 'w', {}, 'r', {}, 'x', {}, 'y', {});
hs = unique(pred.hyp);
for h = reshape(hs, 1, [])
    c = find(pred.hyp == h);
    th.pred(end+1) = struct('src', pred.src(c(1)), ...
        'mode', pred.mode_names{pred.mode(c(1))}, 'w', pred.w(c(1)), ...
        'r', pred.r(c(1)), 'x', ap_mean(pred.x(:, c), 2)', 'y', ap_mean(pred.y(:, c), 2)');
end

th.ra = ra;
th.bp = bp;
th.info = info;
if isfield(info, 'cand')
    th.info = rmfield(info, 'cand');
    th.cand = info.cand;
else
    th.cand = struct('x', [], 'y', [], 'cost', [], 'risk', [], 'code', []);
end
th.look = look;
end


function cfg = cfg_of(st)

cfg = st.cfg;
end
