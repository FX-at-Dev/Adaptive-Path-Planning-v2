function st = ap_stack_init(world, cfg, think)











if nargin < 3, think = false; end

st.tracks  = [];
st.next_id = 1;
st.ctrl    = struct();
st.fsm     = [];
st.traj    = [];
st.info    = struct('feasible', true);
st.bp      = struct('v_cap', cfg.plan.v_max, 'd_max', 2.5, ...
                    'risk_tol', cfg.plan.risk_threshold, 'state', 'CRUISE');
st.ra      = ap_risk_assess(world.ego, [], world.rp, cfg);
st.pred    = [];
st.n_hyp   = 0;
st.think   = think;
st.replan_every = max(1, round(cfg.sim.replan_dt / cfg.sim.dt));
st.cfg     = cfg;
st.rev     = [];
[st.rev]   = ap_reverse([], world.ego, st, cfg, 0, world.rp);
end
