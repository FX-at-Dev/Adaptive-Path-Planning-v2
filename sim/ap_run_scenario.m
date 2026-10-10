function [result, log] = ap_run_scenario(scn, cfg, opts)
























if nargin < 3, opts = struct(); end
if ~isfield(opts, 'verbose'),   opts.verbose   = cfg.verbose; end
if ~isfield(opts, 'snapshots'), opts.snapshots = true; end
if ~isfield(opts, 'think')
    opts.think = isfield(cfg, 'debug') && isfield(cfg.debug, 'think') && cfg.debug.think;
end


cfg.debug.think = opts.think;

world = ap_world_init(scn, cfg);
st    = ap_stack_init(world, cfg, opts.think);

dt = cfg.sim.dt;
N  = round(cfg.sim.t_end / dt);

log = local_init_log(N);
snaps = {};

collided    = false;
reached     = false;
t_reached   = NaN;
collide_t   = NaN;

for k = 1:N
    t = k * dt;


    world = ap_world_step(world, dt, cfg);




    [st, a_cmd, delta_cmd, tk] = ap_stack_tick(st, world, k, cfg);
    world.ego = ap_bicycle_step(world.ego, a_cmd, delta_cmd, dt, cfg, tk.gear);


    [min_clear, hit, worst_id] = ap_clearance_truth(world, cfg);
    if hit && ~collided
        collided  = true;
        collide_t = t;
    end

    [at_goal, d_goal] = ap_goal_reached(world.ego, scn.goal, scn.rp, cfg);
    if ~reached && at_goal
        reached   = true;
        t_reached = t;
    end


    log.t(k)            = t;
    log.x(k)            = world.ego.x;
    log.y(k)            = world.ego.y;
    log.psi(k)          = world.ego.psi;
    log.v(k)            = world.ego.v;
    log.a(k)            = world.ego.a;
    log.delta(k)        = world.ego.delta;
    log.min_clear(k)    = min_clear;
    log.worst_id(k)     = worst_id;
    log.latency_ms(k)   = tk.latency_ms;
    log.state{k}        = st.bp.state;
    log.v_cap(k)        = st.bp.v_cap;
    log.n_tracks(k)     = tk.tdiag.n_track;
    log.n_confirmed(k)  = tk.tdiag.n_confirmed;
    log.n_det(k)        = tk.tdiag.n_det;
    log.n_hypotheses(k) = tk.n_hyp;
    log.feasible(k)     = st.info.feasible;
    log.plan_risk(k)    = st.traj.risk;
    log.d_goal(k)       = d_goal;

    if opts.snapshots && mod(k - 1, 2) == 0
        snaps{end+1} = ap_snapshot(world, st, tk, t, min_clear);
    end

    if opts.verbose && mod(k, round(1/dt)) == 0
        fprintf('  t=%5.1f  v=%4.1f  %-7s  clear=%6.2f  tracks=%2d  lat=%5.1fms\n', ...
                t, world.ego.v, st.bp.state, min_clear, tk.tdiag.n_confirmed, ...
                local_last_finite(log.latency_ms, k));
    end

    if reached
        break;
    end
end

n = k;
log = local_trim_log(log, n);
log.snaps = snaps;

result.name       = scn.name;
result.reached    = reached;
result.collided   = collided;
result.t_reached  = t_reached;
result.t_end      = log.t(end);
result.collide_t  = collide_t;
result.min_clear  = min(log.min_clear);
result.n_steps    = n;
end


function v = local_last_finite(arr, k)
v = NaN;
for i = k:-1:1
    if isfinite(arr(i))
        v = arr(i);
        return;
    end
end
end


function log = local_init_log(N)
z = zeros(N, 1);
log.t = z; log.x = z; log.y = z; log.psi = z; log.v = z; log.a = z;
log.delta = z; log.min_clear = z; log.worst_id = z; log.latency_ms = z;
log.v_cap = z; log.n_tracks = z; log.n_confirmed = z; log.n_det = z;
log.n_hypotheses = z; log.feasible = true(N,1); log.plan_risk = z;
log.d_goal = z;
log.state = cell(N, 1);
end


function log = local_trim_log(log, n)
f = fieldnames(log);
for i = 1:numel(f)
    v = log.(f{i});
    if numel(v) >= n
        log.(f{i}) = v(1:n);
    end
end
end
