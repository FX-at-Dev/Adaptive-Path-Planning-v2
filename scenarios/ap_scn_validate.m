function [ok, why] = ap_scn_validate(scn, cfg)












ok = false;
rp = scn.rp;
hw = cfg.plan.corridor_halfwidth;
need = cfg.ego.width + 1.2;
A = scn.agents;
n = numel(A);
walkers = {'pedestrian', 'cattle'};


s0 = zeros(1, n); d0 = zeros(1, n);
for i = 1:n
    [s0(i), d0(i)] = ap_cart2frenet(rp, A(i).x, A(i).y);
end

for i = 1:n
    if ~any(strcmp(A(i).class, walkers)) && ~strcmp(A(i).mode, 'path') && abs(d0(i)) > hw + 0.5
        why = sprintf('%s #%d parked off the road', A(i).class, A(i).id);
        return;
    end
    if ~any(strcmp(A(i).class, walkers)) && strcmp(A(i).mode, 'path') && abs(d0(i)) > hw + 1.0 && ...
            ~(isfield(scn, 'cross_streets') && any(abs(s0(i) - scn.cross_streets(:, 1)) < 15))
        why = sprintf('%s #%d starts off the road', A(i).class, A(i).id);
        return;
    end
    if A(i).t_spawn <= 0 && hypot(A(i).x - scn.ego.x, A(i).y - scn.ego.y) < 8
        why = sprintf('%s #%d on the ego start', A(i).class, A(i).id);
        return;
    end
end


for i = 1:n
    [ax, ay] = ap_agent_discs(A(i).x, A(i).y, A(i).psi, A(i).props);
    for j = i+1:n
        if A(i).t_spawn ~= A(j).t_spawn && (A(i).t_spawn > 0 || A(j).t_spawn > 0)
            continue;
        end
        [bx, by] = ap_agent_discs(A(j).x, A(j).y, A(j).psi, A(j).props);
        gap = min(min(hypot(ax(:) - bx(:).', ay(:) - by(:).'))) - A(i).props.radius - A(j).props.radius;
        if gap < 0.5
            why = sprintf('%s #%d overlaps %s #%d', A(i).class, A(i).id, A(j).class, A(j).id);
            return;
        end
    end
end


st = find(strcmp({A.mode}, 'static'));
for i = st
    lo = d0(i) - A(i).props.width / 2;
    hi = d0(i) + A(i).props.width / 2;
    for j = st
        if j ~= i && abs(s0(j) - s0(i)) < (A(i).props.length + A(j).props.length) / 2 + 4
            lo = min(lo, d0(j) - A(j).props.width / 2);
            hi = max(hi, d0(j) + A(j).props.width / 2);
        end
    end
    room = max(hw - hi, lo + hw);
    if room < need
        why = sprintf('%s #%d leaves %.1f m, the ego needs %.1f m', A(i).class, A(i).id, room, need);
        return;
    end
end

ok = true;
why = '';
end
