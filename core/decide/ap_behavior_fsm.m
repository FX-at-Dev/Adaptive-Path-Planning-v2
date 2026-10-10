function [fsm, bp] = ap_behavior_fsm(fsm, ego, ra, plan_info, cfg)






























if isempty(fsm) || ~isfield(fsm, 'state')
    fsm.state    = 'CRUISE';
    fsm.hold     = 0;
    fsm.prev     = 'CRUISE';
    fsm.t_in     = 0;
    fsm.stuck    = 0;
end

prev_state = fsm.state;


infeasible = ~isempty(plan_info) && isfield(plan_info, 'feasible') && ~plan_info.feasible;







stopped = ego.v < 0.3;

if stopped
    fsm.stuck = fsm.stuck + 1;
else
    fsm.stuck = 0;
end















unstick = (fsm.stuck > cfg.dec.stuck_cycles) && (ra.min_clear > cfg.dec.stuck_clear);
















ttc_breach = ra.ttc < cfg.dec.ttc_emergency;
if ttc_breach && isfield(ra, 'lead_still') && ra.lead_still
    ttc_breach = ra.lead_gap < ego.v ^ 2 / (2 * abs(cfg.veh.a_min)) + cfg.dec.clear_stop;
end
critical = ttc_breach || ...
           ra.min_clear_moving < cfg.dec.clear_stop || ...
           (ra.min_clear < cfg.dec.clear_stop && ~stopped);



recovering = stopped && infeasible && ra.min_clear > cfg.dec.clear_caution;

yielding  = ra.cross_id > 0 && ra.cross_ttc < cfg.dec.yield_gap;

cautious  = ra.min_clear < cfg.dec.clear_caution || ra.ttc < cfg.dec.ttc_caution;



room = max(ra.free_left, ra.free_right);
can_nudge = room > cfg.plan.safety_margin + 0.5;








following = ra.lead_id > 0 && isfinite(ra.lead_gap) && ...
            ra.lead_speed > 0.8 && ...
            ra.lead_gap < max(12.0, 2.0 * ego.v);

if critical && ~unstick
    state = 'STOP';
    if ra.min_clear < cfg.dec.clear_stop
        why = sprintf('clearance %.2f m under the %.2f m stop limit', ...
                      ra.min_clear, cfg.dec.clear_stop);
    else
        why = sprintf('TTC %.1f s under the %.1f s emergency limit', ...
                      ra.ttc, cfg.dec.ttc_emergency);
    end
elseif unstick
    state = 'CREEP';
    why = sprintf('stationary %.1f s with %.1f m clear, edging out', ...
                  fsm.stuck * cfg.sim.replan_dt, ra.min_clear);
elseif infeasible && ~stopped









    state = local_escalate(prev_state);
    why = sprintf('no plan within risk tolerance, escalating from %s', prev_state);
elseif recovering
    state = 'CREEP';
    why = sprintf('stopped without a plan but %.1f m clear, creeping', ra.min_clear);
elseif yielding
    state = 'YIELD';
    why = sprintf('agent #%d crossing, %.1f s to conflict', ra.cross_id, ra.cross_ttc);
elseif cautious
    if can_nudge && ra.n_near <= 4
        state = 'NUDGE';
        if ra.free_left >= ra.free_right
            why = sprintf('%.1f m clearance, %.1f m free left, easing around', ...
                          ra.min_clear, ra.free_left);
        else
            why = sprintf('%.1f m clearance, %.1f m free right, easing around', ...
                          ra.min_clear, ra.free_right);
        end
    else
        state = 'CREEP';
        why = sprintf('tight: %.1f m clearance, %d agents near, %.1f m room', ...
                      ra.min_clear, ra.n_near, room);
    end
elseif following
    state = 'FOLLOW';
    why = sprintf('following #%d, %.1f m ahead at %.1f m/s', ...
                  ra.lead_id, ra.lead_gap, ra.lead_speed);
else
    state = 'CRUISE';
    why = 'clear road';
end





sev_new = local_severity(state);
sev_old = local_severity(prev_state);

if sev_new >= sev_old
    fsm.hold = local_hold_cycles(state);
else
    if fsm.hold > 0
        fsm.hold = fsm.hold - 1;
        why      = sprintf('holding %s, would relax to %s: %s', prev_state, state, why);
        state    = prev_state;
    else
        fsm.hold = local_hold_cycles(state);
    end
end

if strcmp(state, prev_state)
    fsm.t_in = fsm.t_in + 1;
else
    fsm.t_in = 0;
end

fsm.prev  = prev_state;
fsm.state = state;










hw = cfg.plan.corridor_halfwidth;

switch state
    case 'CRUISE'
        bp.v_cap    = cfg.plan.v_max;
        bp.d_max    = 0.80 * hw;
        bp.risk_tol = cfg.plan.risk_threshold;

    case 'FOLLOW'


        desired = max(0, ra.lead_speed);
        if isfinite(ra.lead_gap)
            desired = desired + 0.35 * (ra.lead_gap - max(6.0, 1.5 * ego.v));
        end
        bp.v_cap    = min(cfg.plan.v_max, max(0, desired));
        bp.d_max    = 0.80 * hw;
        bp.risk_tol = cfg.plan.risk_threshold;

    case 'NUDGE'


        bp.v_cap    = cfg.dec.caution_scale * cfg.plan.v_max;
        bp.d_max    = 1.00 * hw;
        bp.risk_tol = 0.8 * cfg.plan.risk_threshold;

    case 'YIELD'


        bp.v_cap    = min(cfg.dec.creep_speed * 1.4, cfg.plan.v_max);
        bp.d_max    = 0.70 * hw;
        bp.risk_tol = 0.5 * cfg.plan.risk_threshold;

    case 'CREEP'
        bp.v_cap    = cfg.dec.creep_speed;
        bp.d_max    = 0.95 * hw;
        if unstick

            bp.risk_tol = cfg.dec.stuck_risk_scale * cfg.plan.risk_threshold;
        else
            bp.risk_tol = 0.6 * cfg.plan.risk_threshold;
        end

    case 'STOP'






        bp.v_cap    = 0;
        bp.d_max    = 0.60 * hw;
        bp.risk_tol = cfg.plan.risk_threshold;

    otherwise
        error('ap_behavior_fsm:state', 'Unhandled state "%s".', state);
end






switch state
    case 'NUDGE', bp.lane_w = 0.15;
    case 'CREEP', bp.lane_w = 0.3;
    otherwise,    bp.lane_w = 1.0;
end












bp.hold = zeros(0, 3);
if isfield(ra, 'still_ahead') && ~isempty(ra.still_ahead)
    bp.hold = [ra.still_ahead(:, 1) - cfg.dec.standoff, ra.still_ahead(:, 2:3)];
end
if ra.lead_id > 0 && isfield(ra, 'lead_still') && ra.lead_still && ~isempty(ra.lead_box)
    if ra.lead_gap < cfg.dec.standoff + 3 && ~strcmp(state, 'STOP')
        bp.v_cap = max(bp.v_cap, cfg.dec.pullout_speed);
        why = sprintf('%s; %.0f m back from #%d, pulling out round it when clear', why, ra.lead_gap, ra.lead_id);
    end
end






if isfield(ra, 'caution_gap') && isfinite(ra.caution_gap)
    v_c = sqrt(2 * cfg.dec.caution_decel * max(ra.caution_gap - cfg.dec.caution_margin, 0));
    if v_c < bp.v_cap
        bp.v_cap = v_c;
        why = sprintf('%s; slowing for #%d %.0f m ahead until it is confirmed', why, ra.caution_id, ra.caution_gap);
    end
end

bp.state   = state;
bp.unstick = unstick;
bp.reason  = why;
end


function next = local_escalate(state)






switch state
    case {'CRUISE', 'FOLLOW'}, next = 'NUDGE';
    case {'NUDGE', 'YIELD'},   next = 'CREEP';
    case 'CREEP',              next = 'STOP';
    case 'STOP',               next = 'STOP';
    otherwise,                 next = 'CREEP';
end
end


function s = local_severity(state)

switch state
    case 'CRUISE', s = 0;
    case 'FOLLOW', s = 1;
    case 'NUDGE',  s = 2;
    case 'YIELD',  s = 3;
    case 'CREEP',  s = 3;
    case 'STOP',   s = 4;
    otherwise,     s = 0;
end
end


function n = local_hold_cycles(state)



switch state
    case 'STOP',   n = 8;
    case 'YIELD',  n = 6;
    case 'CREEP',  n = 6;
    case 'NUDGE',  n = 5;
    case 'FOLLOW', n = 3;
    otherwise,     n = 0;
end
end
