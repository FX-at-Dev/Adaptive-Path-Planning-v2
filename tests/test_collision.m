function test_collision()







cfg = ap_config();


[cx, cy, r] = ap_ego_discs(0, 0, 0, cfg);
ap_assert_true(numel(cx) == cfg.ego.n_discs, 'one centre per disc');
ap_assert_close(cy, zeros(1, cfg.ego.n_discs), 1e-12, 'centres on the axis');


seg = cfg.ego.length / cfg.ego.n_discs;
ap_assert_close(r, sqrt((seg/2)^2 + (cfg.ego.width/2)^2), 1e-12, 'covering radius');



rear_overhang = 0.5 * (cfg.ego.length - cfg.ego.wheelbase);
ap_assert_true(max(cx) + r >= cfg.ego.length - rear_overhang - 1e-9, ...
    'discs must cover the front bumper');
ap_assert_true(min(cx) - r <= -rear_overhang + 1e-9, ...
    'discs must cover the rear bumper');


[cx90, cy90] = ap_ego_discs(0, 0, pi/2, cfg);
ap_assert_close(cx90, zeros(1, cfg.ego.n_discs), 1e-12, 'rotated centres x');
ap_assert_close(cy90, cx, 1e-12, 'rotated centres y');




pred = local_pred(0.2, 21, 10.0, 0.0, 0.5, 1.0);
te   = (0:0.2:2)';
n    = numel(te);

[collides, min_clear, risk] = ap_collision_check( ...
    zeros(n,1), zeros(n,1), zeros(n,1), te, pred, cfg);

expect = 10.0 - max(cx) - (r + cfg.plan.safety_margin + 0.5);
ap_assert_true(~collides, 'a target 10 m ahead must not collide');
ap_assert_close(min_clear, expect, 1e-9, 'hand-computed clearance');
ap_assert_close(risk, 0, 1e-12, 'no risk when clear');


pred = local_pred(0.2, 21, 2.5, 0.0, 0.5, 1.0);
[collides, min_clear, risk] = ap_collision_check( ...
    zeros(n,1), zeros(n,1), zeros(n,1), te, pred, cfg);
ap_assert_true(collides, 'a target inside the footprint must collide');
ap_assert_true(min_clear < 0, 'penetration must report negative clearance');
ap_assert_close(risk, 1.0, 1e-12, 'certain hypothesis gives full risk');



pred = local_pred(0.2, 21, 2.5, 0.0, 0.5, 0.3);
far  = local_pred(0.2, 21, 40.0, 0.0, 0.5, 0.7);
pred = local_join(pred, far);

[collides, ~, risk] = ap_collision_check( ...
    zeros(n,1), zeros(n,1), zeros(n,1), te, pred, cfg);
ap_assert_true(collides, 'conflict with the unlikely hypothesis still counts');
ap_assert_close(risk, 0.3, 1e-12, 'risk equals the conflicting weight only');




props = ap_agent_props('bus');
ap_assert_true(props.n_discs >= 3, 'a bus should need several discs');
[bx, by] = ap_agent_discs(3.0, 0.0, 0.0, props);

A = props.n_discs;
K = 21;
pred = struct();
pred.t   = (0:0.2:4)';
pred.x   = ones(K,1) * bx;
pred.y   = ones(K,1) * by;
pred.r   = props.radius * ones(1, A);
pred.w   = 0.4 * ones(1, A);
pred.hyp = ones(1, A);

[collides, ~, risk] = ap_collision_check( ...
    zeros(n,1), zeros(n,1), zeros(n,1), te, pred, cfg);
ap_assert_true(collides, 'bus across the bonnet must collide');
ap_assert_close(risk, 0.4, 1e-12, 'multi-disc agent counted once');




K = 21;
tt = (0:0.2:4)';
ax = 30 - 13.5 * tt;
pred = struct();
pred.t   = tt;
pred.x   = ax;
pred.y   = zeros(K,1);
pred.r   = 0.5;
pred.w   = 1.0;
pred.hyp = 1;

te_short = (0:0.2:1.0)';
[c_short, ~, ~] = ap_collision_check(zeros(numel(te_short),1), ...
    zeros(numel(te_short),1), zeros(numel(te_short),1), te_short, pred, cfg);
ap_assert_true(~c_short, 'short horizon must not see the later conflict');

te_long = (0:0.2:3.0)';
[c_long, ~, ~] = ap_collision_check(zeros(numel(te_long),1), ...
    zeros(numel(te_long),1), zeros(numel(te_long),1), te_long, pred, cfg);
ap_assert_true(c_long, 'longer horizon must catch the conflict at t = 2 s');


empty_pred = struct('t', (0:0.2:4)', 'x', zeros(21,0), 'y', zeros(21,0), ...
                    'r', zeros(1,0), 'w', zeros(1,0), 'hyp', zeros(1,0));
[c, mc, rk] = ap_collision_check(zeros(n,1), zeros(n,1), zeros(n,1), te, empty_pred, cfg);
ap_assert_true(~c && isinf(mc) && rk == 0, 'empty world is collision free');
end


function pred = local_pred(dt, K, x0, y0, radius, weight)

pred.t   = (0:dt:(K-1)*dt)';
pred.x   = x0 * ones(K, 1);
pred.y   = y0 * ones(K, 1);
pred.r   = radius;
pred.w   = weight;
pred.hyp = 1;
end


function p = local_join(a, b)

p     = a;
p.x   = [a.x,   b.x];
p.y   = [a.y,   b.y];
p.r   = [a.r,   b.r];
p.w   = [a.w,   b.w];
p.hyp = [a.hyp, b.hyp + max(a.hyp)];
end
