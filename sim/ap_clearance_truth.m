function [min_clear, collided, worst_id] = ap_clearance_truth(world, cfg)









min_clear = Inf;
collided  = false;
worst_id  = 0;

[ecx, ecy, erad] = ap_ego_discs(world.ego.x, world.ego.y, world.ego.psi, cfg);

for k = 1:numel(world.agents)
    a = world.agents(k);
    if ~a.active
        continue;
    end

    [acx, acy] = ap_agent_discs(a.x, a.y, a.psi, a.props);

    d = Inf;
    for i = 1:numel(ecx)
        d = min(d, min(sqrt((ecx(i) - acx).^2 + (ecy(i) - acy).^2)));
    end

    clear_k = d - erad - a.props.radius;
    if clear_k < min_clear
        min_clear = clear_k;
        worst_id  = a.id;
    end
end

collided = (min_clear < 0);
end
