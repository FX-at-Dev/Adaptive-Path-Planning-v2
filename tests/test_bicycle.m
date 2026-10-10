function test_bicycle()







cfg = ap_config();
L   = cfg.ego.wheelbase;
dt  = cfg.sim.dt;


delta = 0.20;
v     = 5.0;
Rc    = L / tan(delta);

ego = struct('x', 0, 'y', 0, 'psi', 0, 'v', v, 'delta', delta);

n = 40;
for k = 1:n

    ego = ap_bicycle_step(ego, 0, delta, dt, cfg);
end

t   = n * dt;
psi = v * t / Rc;
ap_assert_close(ego.psi, ap_wrap_pi(psi), 1e-7, 'circle heading');
ap_assert_close(ego.x,   Rc * sin(psi),    1e-6, 'circle x');
ap_assert_close(ego.y,   Rc * (1 - cos(psi)), 1e-6, 'circle y');
ap_assert_close(ego.v,   v, 1e-12, 'circle speed held');



ap_assert_true(ego.y > 0, 'positive steer must yield a left turn');


ego = struct('x', 0, 'y', 0, 'psi', 0, 'v', 6.0, 'delta', 0);
for k = 1:20
    ego = ap_bicycle_step(ego, 0, 0, dt, cfg);
end
ap_assert_close(ego.x, 6.0 * 20 * dt, 1e-9, 'straight distance');
ap_assert_close(ego.y, 0, 1e-12, 'straight lateral drift');


ego = struct('x', 0, 'y', 0, 'psi', 0, 'v', 0, 'delta', 0);
a   = 2.0;
n   = 25;
for k = 1:n
    ego = ap_bicycle_step(ego, a, 0, dt, cfg);
end
t = n * dt;
ap_assert_close(ego.v, a*t,           1e-9,  'accel final speed');
ap_assert_close(ego.x, 0.5*a*t^2,     1e-9,  'accel distance');




ego = struct('x', 0, 'y', 0, 'psi', 0, 'v', 0.1, 'delta', 0);
ego = ap_bicycle_step(ego, -7.0, 0, dt, cfg);
ap_assert_close(ego.v, 0, 1e-12, 'braking clamps at zero speed');
ap_assert_true(ego.x > 0 && ego.x < 0.1*dt + 1e-9, ...
    'stopping step must travel forward but less than a constant-speed step');


x_stop = ego.x;
ego = ap_bicycle_step(ego, -7.0, 0, dt, cfg);
ap_assert_close(ego.v, 0, 1e-12, 'stays stopped');
ap_assert_close(ego.x, x_stop, 1e-12, 'no reverse creep');




ego = struct('x', 0, 'y', 0, 'psi', 0, 'v', 3.0, 'delta', 0);
ego = ap_bicycle_step(ego, 0, cfg.veh.delta_max, dt, cfg);
ap_assert_close(ego.delta, cfg.veh.delta_rate * dt, 1e-12, 'steering rate limit');


ego = struct('x', 0, 'y', 0, 'psi', 0, 'v', 3.0, 'delta', 0);
ego = ap_bicycle_step(ego, 99, 0, dt, cfg);
ap_assert_close(ego.a, cfg.veh.a_max, 1e-12, 'acceleration saturates');
end
