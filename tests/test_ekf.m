function test_ekf()








cfg = ap_config();
dt  = 0.1;


x = [3.2; -1.7; 8.0; 0.4; 0.25];
[~, F] = ap_ctrv_motion(x, dt);
ap_assert_close(F, local_num_jac(x, dt), 1e-6, 'CTRV Jacobian (turning)');

x = [0; 0; 12.0; -0.9; -0.6];
[~, F] = ap_ctrv_motion(x, dt);
ap_assert_close(F, local_num_jac(x, dt), 1e-6, 'CTRV Jacobian (hard turn)');


x = [1.0; 2.0; 6.0; 0.3; 1e-6];
[~, F] = ap_ctrv_motion(x, dt);
ap_assert_close(F, local_num_jac(x, dt), 1e-6, 'CTRV Jacobian (straight)');



ap_assert_true(abs(F(2,5)) > 1e-9, 'straight branch must retain d(py)/d(omega)');


eps_w = 1e-9;
xa = [1.0; 2.0; 6.0; 0.3;  1e-4 + eps_w];
xb = [1.0; 2.0; 6.0; 0.3;  1e-4 - eps_w];
ap_assert_close(ap_ctrv_motion(xa, dt), ap_ctrv_motion(xb, dt), 1e-8, ...
                 'motion continuous across the omega branch');


x = [0; 0; 10; 0; 0];
for k = 1:20
    x = ap_ctrv_motion(x, dt);
end
ap_assert_close(x(1), 10 * 20 * dt, 1e-9, 'CTRV straight distance');
ap_assert_close(x(2), 0, 1e-12, 'CTRV straight has no drift');



v = 8; w = 0.25;
x = [0; 0; v; 0; w];
n = 30;
for k = 1:n
    x = ap_ctrv_motion(x, dt);
end
Rc  = v / w;
psi = w * n * dt;
ap_assert_close(x(1), Rc*sin(psi),     1e-9, 'CTRV circle x');
ap_assert_close(x(2), Rc*(1-cos(psi)), 1e-9, 'CTRV circle y');
ap_assert_close(x(3), v, 1e-12, 'CTRV holds speed');
ap_assert_close(x(5), w, 1e-12, 'CTRV holds turn rate');


ap_rng(4242);

x_true = [0; 0; 7.0; 0.2; 0.12];
sigma  = 0.5;
R      = sigma^2 * eye(2);
N      = 160;




x_est = [x_true(1); x_true(2); 0; 0; 0];
P     = diag([cfg.fuse.p0_pos, cfg.fuse.p0_pos, cfg.fuse.p0_vel, ...
              cfg.fuse.p0_yaw, cfg.fuse.p0_omega]);

err_meas  = zeros(N, 1);
err_pos   = zeros(N, 1);
err_head  = zeros(N, 1);
err_speed = zeros(N, 1);
err_omega = zeros(N, 1);

for k = 1:N
    x_true = ap_ctrv_motion(x_true, dt);
    z = x_true(1:2) + sigma * randn(2, 1);

    [x_est, P] = ap_ekf_predict(x_est, P, dt, cfg);
    [x_est, P, ~, valid] = ap_ekf_update(x_est, P, z, R, []);
    ap_assert_true(valid, 'ungated update must always apply');

    err_meas(k)  = norm(z          - x_true(1:2));
    err_pos(k)   = norm(x_est(1:2) - x_true(1:2));
    err_head(k)  = ap_wrap_pi(x_est(4) - x_true(4));
    err_speed(k) = x_est(3) - x_true(3);
    err_omega(k) = x_est(5) - x_true(5);
end











w = round(N/2):N;

ap_assert_true(mean(err_pos(w)) < 0.50, ...
    'position MAE %.3f m too high', mean(err_pos(w)));

ap_assert_true(mean(abs(err_head(w))) < 0.13, ...
    'heading MAE %.4f rad too high', mean(abs(err_head(w))));
ap_assert_true(abs(mean(err_head(w))) < 0.04, ...
    'heading bias %.4f rad -- estimator is systematically off', mean(err_head(w)));

ap_assert_true(mean(abs(err_speed(w))) < 0.40, ...
    'speed MAE %.3f m/s too high', mean(abs(err_speed(w))));
ap_assert_true(abs(mean(err_speed(w))) < 0.15, ...
    'speed bias %.3f m/s', mean(err_speed(w)));

ap_assert_true(mean(abs(err_omega(w))) < 0.13, ...
    'turn-rate MAE %.4f rad/s too high', mean(abs(err_omega(w))));
ap_assert_true(abs(mean(err_omega(w))) < 0.03, ...
    'turn-rate bias %.4f rad/s', mean(err_omega(w)));



rms_meas = sqrt(mean(err_meas(w).^2));
rms_est  = sqrt(mean(err_pos(w).^2));
ap_assert_true(rms_est < 0.75 * rms_meas, ...
    'filtered RMS %.3f must improve on measurement RMS %.3f', rms_est, rms_meas);


ap_assert_close(P, P.', 1e-12, 'covariance symmetric');
ap_assert_true(all(eig(P) > 0), 'covariance positive definite');


x_before = x_est;
P_before = P;
z_bad = x_est(1:2) + [40; 40];
[x_after, P_after, nis, valid] = ap_ekf_update(x_est, P, z_bad, R, cfg.fuse.gate_chi2);
ap_assert_true(~valid, 'a 40 m outlier must fail the gate');
ap_assert_true(nis > cfg.fuse.gate_chi2, 'outlier NIS must exceed the gate');
ap_assert_close(x_after, x_before, 0, 'rejected update leaves state unchanged');
ap_assert_close(P_after, P_before, 0, 'rejected update leaves covariance unchanged');


[~, ~, nis_ok, valid_ok] = ap_ekf_update(x_est, P, x_est(1:2), R, cfg.fuse.gate_chi2);
ap_assert_true(valid_ok && nis_ok < 1e-9, 'zero-innovation update must pass the gate');
end


function J = local_num_jac(x, dt)

h = 1e-6;
J = zeros(5, 5);
for i = 1:5
    xp = x; xp(i) = xp(i) + h;
    xm = x; xm(i) = xm(i) - h;
    fp = ap_ctrv_motion(xp, dt);
    fm = ap_ctrv_motion(xm, dt);
    J(:, i) = (fp - fm) / (2*h);
end
end
