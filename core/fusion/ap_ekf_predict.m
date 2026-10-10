function [x, P] = ap_ekf_predict(x, P, dt, cfg)











[x, F] = ap_ctrv_motion(x, dt);

psi = x(4);


Ga = [0.5 * dt^2 * cos(psi);
      0.5 * dt^2 * sin(psi);
      dt;
      0;
      0];


Gw = [0;
      0;
      0;
      0.5 * dt^2;
      dt];

Q = Ga * (cfg.fuse.q_accel^2)   * Ga.' + ...
    Gw * (cfg.fuse.q_yawrate^2) * Gw.';

P = F * P * F.' + Q;
P = 0.5 * (P + P.');
end
