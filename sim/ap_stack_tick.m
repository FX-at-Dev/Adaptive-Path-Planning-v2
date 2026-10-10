function [st, a_cmd, delta_cmd, tk] = ap_stack_tick(st, world, k, cfg, dets)























rp = world.rp;


if nargin < 5
    dets = ap_sense(world, cfg);
end
[st.tracks, st.next_id, tdiag] = ap_tracker_step(st.tracks, dets, cfg.sim.dt, st.next_id, cfg);




was_reversing = ~strcmp(st.rev.phase, 'idle');
[st.rev, a_rev, d_rev, reversing, why_rev] = ap_reverse(st.rev, world.ego, st, cfg, cfg.sim.dt, rp);
if reversing
    st.bp.state  = 'REVERSE';
    st.bp.reason = why_rev;
    st.bp.v_cap  = 0;
    tk.dets = dets; tk.tdiag = tdiag; tk.did_replan = false; tk.latency_ms = NaN;
    tk.n_hyp = st.n_hyp; tk.look = []; tk.gear = -1;
    a_cmd = a_rev; delta_cmd = d_rev;
    return;
end


did_replan = (mod(k - 1, st.replan_every) == 0) || was_reversing;
if did_replan
    st.ra             = ap_risk_assess(world.ego, st.tracks, rp, cfg);
    [st.fsm, st.bp]   = ap_behavior_fsm(st.fsm, world.ego, st.ra, st.info, cfg);
    st.bp             = ap_arrive(st.bp, world.ego, world.goal, rp, cfg);
    st.pred           = ap_predict_intent(st.tracks, cfg);
    [st.traj, st.info] = ap_lattice_plan(world.ego, rp, st.pred, st.bp, cfg);
    latency = st.info.latency_ms;
    st.n_hyp = 0;
    if ~isempty(st.pred.hyp)
        st.n_hyp = numel(unique(st.pred.hyp));
    end
else
    latency = NaN;
end




[a_cmd, delta_cmd, st.ctrl] = ap_controller(world.ego, st.traj, st.ctrl, cfg);

tk.dets       = dets;
tk.tdiag      = tdiag;
tk.did_replan = did_replan;
tk.latency_ms = latency;
tk.n_hyp      = st.n_hyp;
tk.look       = [];
tk.gear       = 1;
if world.ego.v < 0
    tk.gear = -1;
end
if st.think

    tk.look = [world.ego.x + st.ctrl.Ld * cos(world.ego.psi + st.ctrl.alpha), ...
               world.ego.y + st.ctrl.Ld * sin(world.ego.psi + st.ctrl.alpha)];
end
end
