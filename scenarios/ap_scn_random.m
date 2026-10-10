function [scn, cfg] = ap_scn_random(scn, cfg, seed, draw)














if isempty(seed)
    return;
end
cfg.sim.seed = mod(round(seed), 2^31 - 1);
for attempt = 0:59
    ap_rng(cfg.sim.seed + 7919 * attempt);
    R.u = @(a, b) a + (b - a) * rand();
    R.pick = @(c) c{1 + floor(rand() * numel(c) * 0.999999)};
    R.coin = @(p) rand() < p;
    R.int = @(a, b) a + floor(rand() * (b - a + 1) * 0.999999);
    A = draw(scn.rp, cfg, R);

    if ~isfield(cfg, 'scn') || ~isfield(cfg.scn, 'traffic') || cfg.scn.traffic
        A = [A, ap_scn_traffic(scn, cfg, R, max([0, A.id]) + 1)];
    end
    scn.agents = A;
    [ok, why] = ap_scn_validate(scn, cfg);
    if ok
        scn.seed = seed;
        scn.desc = sprintf('%s (random layout, seed %d)', scn.desc, round(seed));
        return;
    end
    last = why;
end
error('ap_scn_random:invalid', 'no valid layout for %s with seed %d (%s)', scn.name, round(seed), last);
end
