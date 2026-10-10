function [rv, a_cmd, delta_cmd, active, why] = ap_reverse(rv, ego, st, cfg, dt, rp)
























if isempty(rv)
    rv = struct('phase', 'idle', 'stuck_t', 0, 'x0', 0, 'y0', 0, 't', 0, ...
                'cool', 0, 'steer', 0, 'side', 0);
end
a_cmd = 0; delta_cmd = 0; active = false; why = '';

switch rv.phase
    case 'idle'
        rv.cool = max(0, rv.cool - dt);
        if local_boxed(ego, st, cfg)
            rv.stuck_t = rv.stuck_t + dt;
        else
            rv.stuck_t = 0;
        end
        if rv.stuck_t >= cfg.dec.reverse_after && rv.cool <= 0 && ...
                local_clear_behind(ego, st.tracks, cfg, cfg.dec.reverse_dist + 1.5) && ...
                local_on_road(ego, rp, cfg, cfg.dec.reverse_dist + 0.5)
            rv.phase = 'back';
            rv.x0 = ego.x; rv.y0 = ego.y; rv.t = 0;

            if st.ra.free_right >= st.ra.free_left
                rv.side = -1;
            else
                rv.side = 1;
            end
        else
            return;
        end
end

rv.t = rv.t + dt;
active = true;
switch rv.phase
    case 'back'
        moved = hypot(ego.x - rv.x0, ego.y - rv.y0);
        if moved >= cfg.dec.reverse_dist || rv.t > 8 || ~local_clear_behind(ego, st.tracks, cfg, 1.5) || ...
                ~local_on_road(ego, rp, cfg, 0.8)
            rv.phase = 'stop';
        end
        a_cmd = min(max(2.0 * (-cfg.dec.reverse_speed - ego.v), -1.5), 1.5);
        rv.steer = local_steer(ego, rv.side, rp, cfg);
        delta_cmd = rv.steer;
        why = sprintf('boxed in: backing up %.1f of %.0f m to steer round', moved, cfg.dec.reverse_dist);
    case 'stop'
        a_cmd = 2.0;
        delta_cmd = rv.steer;
        why = 'backed up; stopping to drive on';
        if abs(ego.v) < 0.02
            rv.phase = 'idle';
            rv.stuck_t = 0;
            rv.cool = cfg.dec.reverse_cooldown;
            active = false;
        end
end
end


function b = local_boxed(ego, st, cfg)

b = false;
if abs(ego.v) > 0.1 || isempty(st.ra) || ~isfield(st, 'info') || isempty(st.info)
    return;
end

if isfield(st.bp, 'reason') && strncmp(st.bp.reason, 'at the target', 13)
    return;
end
no_way = isfield(st.info, 'n_feasible') && st.info.n_feasible == 0;
if ~no_way && isfield(st, 'traj') && ~isempty(st.traj) && isfield(st.traj, 'v')
    no_way = max(abs(st.traj.v)) < 0.3;
end


close = (isfield(st.ra, 'min_clear_still') && st.ra.min_clear_still < cfg.dec.reverse_clear) || ...
        (isfield(st.ra, 'lead_still') && st.ra.lead_still && st.ra.lead_gap < cfg.dec.standoff - 1);
b = no_way && close;
end


function ok = local_clear_behind(ego, tracks, cfg, reach)

ok = true;
c = cos(ego.psi); s = sin(ego.psi);
rear = -0.5 * (cfg.ego.length - cfg.ego.wheelbase);
half = 0.5 * cfg.ego.width + 0.5;
for i = 1:numel(tracks)
    t = tracks(i);
    if strcmp(t.status, 'tentative'), continue; end
    fp = ap_footprint(t, cfg);
    dx = [fp.corners(:, 1); fp.cx] - ego.x;
    dy = [fp.corners(:, 2); fp.cy] - ego.y;
    u = c * dx + s * dy;
    v = -s * dx + c * dy;
    if any(u < rear + 0.3 & u > rear - reach & abs(v) < half)
        ok = false;
        return;
    end
end
end


function ok = local_on_road(ego, rp, cfg, back)

ok = true;
if isempty(rp), return; end
c = cos(ego.psi); s = sin(ego.psi);
rear = -0.5 * (cfg.ego.length - cfg.ego.wheelbase) - back;
for side = [-1, 1]
    px = ego.x + c * rear - s * side * 0.5 * cfg.ego.width;
    py = ego.y + s * rear + c * side * 0.5 * cfg.ego.width;
    [sp, dp] = ap_cart2frenet(rp, px, py);
    hw = interp1(rp.s, rp.halfwidth, min(max(sp, 0), rp.length), 'linear');
    if abs(dp) > hw + cfg.plan.road_overhang
        ok = false;
        return;
    end
end
end


function delta = local_steer(ego, side, rp, cfg)





if isempty(rp)
    delta = -side * cfg.dec.reverse_steer;
    return;
end
[~, ~, ~, psi_road] = ap_cart2frenet(rp, ego.x, ego.y);
err = psi_road + side * cfg.dec.reverse_angle - ego.psi;
err = atan2(sin(err), cos(err));


delta = min(max(-2.0 * err, -cfg.dec.reverse_steer), cfg.dec.reverse_steer);
end
