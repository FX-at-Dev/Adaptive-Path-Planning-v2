function [traj, info] = ap_hybrid_astar(ego, rp, pred, bp, cfg)

























t_start = tic;

traj = [];
info = struct('expanded', 0, 'found', false, 'latency_ms', 0);

H = cfg.plan.hybrid;


[s0, d0] = ap_cart2frenet(rp, ego.x, ego.y);
s_goal   = min(s0 + H.lookahead, rp.length);
if s_goal - s0 < 2.0
    info.latency_ms = toc(t_start) * 1000;
    return;
end



d_goal = max(min(0, bp.d_max), -bp.d_max);
[gx, gy] = ap_frenet2cart(rp, s_goal, d_goal);

grid = ap_occupancy_grid(ego, pred, cfg);


nyaw = H.yaw_bins;
cap  = H.max_expand;

nx    = zeros(cap, 1);  ny = zeros(cap, 1);  npsi = zeros(cap, 1);
ng    = zeros(cap, 1);  nf = zeros(cap, 1);  nt   = zeros(cap, 1);
npar  = zeros(cap, 1);  nst = zeros(cap, 1);




visited = false(grid.n, grid.n, nyaw);

nx(1) = ego.x;  ny(1) = ego.y;  npsi(1) = ego.psi;
ng(1) = 0;      nt(1) = 0;      npar(1) = 0;  nst(1) = 0;
nf(1) = hypot(gx - ego.x, gy - ego.y);

open_idx = 1;
n_used   = 1;
best_goal = 0;

v_f  = max(H.speed, 0.5);
step = H.step;
dt_p = step / v_f;

while ~isempty(open_idx)
    [~, oi] = min(nf(open_idx));
    cur = open_idx(oi);
    open_idx(oi) = [];


    if hypot(nx(cur) - gx, ny(cur) - gy) < H.goal_tol_xy
        best_goal = cur;
        break;
    end

    [ix, iy, iyaw] = local_index(nx(cur), ny(cur), npsi(cur), grid, nyaw);
    if ix < 1 || iy < 1 || ix > grid.n || iy > grid.n
        continue;
    end
    if visited(ix, iy, iyaw)
        continue;
    end
    visited(ix, iy, iyaw) = true;

    info.expanded = info.expanded + 1;
    if info.expanded > cap - numel(H.steer_set) - 1
        break;
    end


    for si = 1:numel(H.steer_set)
        delta = H.steer_set(si);



        x = nx(cur); y = ny(cur); psi = npsi(cur);
        half = step / 2;
        ok = true;
        for sub = 1:2
            psi = psi + (half / cfg.ego.wheelbase) * tan(delta);
            x   = x + half * cos(psi);
            y   = y + half * sin(psi);
        end
        psi = ap_wrap_pi(psi);

        t_new = nt(cur) + dt_p;
        if t_new > H.max_time
            continue;
        end









        [s_n, d_n] = ap_cart2frenet(rp, x, y);
        hw_n = interp1(rp.s, rp.halfwidth, min(max(s_n, 0), rp.length), 'linear');
        if abs(d_n) > hw_n
            continue;
        end
        if s_n < s0 - 1.0
            continue;
        end


        if local_blocked(x, y, t_new, grid)
            continue;
        end

        [jx, jy, jyaw] = local_index(x, y, psi, grid, nyaw);
        if jx < 1 || jy < 1 || jx > grid.n || jy > grid.n || visited(jx, jy, jyaw)
            continue;
        end



        g = ng(cur) + step ...
            + H.w_steer  * abs(delta) * step ...
            + H.w_change * abs(delta - nst(cur)) ...
            + H.w_offset * abs(d_n) * step / max(hw_n, 0.5);

        h = hypot(gx - x, gy - y);

        n_used = n_used + 1;
        if n_used > cap
            n_used = cap;
            break;
        end
        nx(n_used)   = x;    ny(n_used)  = y;   npsi(n_used) = psi;
        ng(n_used)   = g;    nf(n_used)  = g + H.w_heur * h;
        nt(n_used)   = t_new;
        npar(n_used) = cur;  nst(n_used) = delta;

        open_idx(end+1) = n_used;
    end
end

if best_goal == 0
    info.latency_ms = toc(t_start) * 1000;
    return;
end


chain = best_goal;
while npar(chain(1)) > 0
    chain = [npar(chain(1)), chain];
end

px = nx(chain);
py = ny(chain);
pp = npsi(chain);
pt = nt(chain);

if numel(px) < 3
    info.latency_ms = toc(t_start) * 1000;
    return;
end



tv = (0:cfg.pred.dt:min(pt(end), cfg.pred.horizon))';
if numel(tv) < 3
    info.latency_ms = toc(t_start) * 1000;
    return;
end

[pt_u, iu] = unique(pt);
traj.t     = tv;
traj.x     = interp1(pt_u, px(iu), tv, 'linear', 'extrap');
traj.y     = interp1(pt_u, py(iu), tv, 'linear', 'extrap');
traj.psi   = ap_wrap_pi(interp1(pt_u, unwrap(pp(iu)), tv, 'linear', 'extrap'));
traj.v     = v_f * ones(size(tv));
traj.a     = zeros(size(tv));
traj.kappa = tan(interp1(pt_u, nst(chain(iu)), tv, 'linear', 'extrap')) / cfg.ego.wheelbase;

[traj.s, traj.d] = deal(zeros(size(tv)));
for k = 1:numel(tv)
    [traj.s(k), traj.d(k)] = ap_cart2frenet(rp, traj.x(k), traj.y(k));
end




[collides, min_clear, risk] = ap_collision_check( ...
    traj.x, traj.y, traj.psi, tv, pred, cfg);

traj.cost      = ng(best_goal);
traj.risk      = risk;
traj.min_clear = min_clear;
traj.T         = tv(end);
traj.d_target  = d_goal;
traj.v_target  = v_f;
traj.emergency = false;
traj.feasible  = ~collides || min_clear > 0;
traj.hybrid    = true;

info.found      = true;
info.latency_ms = toc(t_start) * 1000;
end


function [ix, iy, iyaw] = local_index(x, y, psi, grid, nyaw)

ix = floor((x - grid.x0) / grid.res) + 1;
iy = floor((y - grid.y0) / grid.res) + 1;
iyaw = mod(floor((psi + pi) / (2*pi) * nyaw), nyaw) + 1;
end


function tf = local_blocked(x, y, t, grid)

ix = floor((x - grid.x0) / grid.res) + 1;
iy = floor((y - grid.y0) / grid.res) + 1;
if ix < 1 || iy < 1 || ix > grid.n || iy > grid.n
    tf = true;
    return;
end
it = min(max(floor(t / grid.dt) + 1, 1), grid.nt);
tf = grid.occ(ix, iy, it);
end
