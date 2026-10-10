function ego = ap_bicycle_step(ego, a_cmd, delta_cmd, dt, cfg, gear)























L = cfg.ego.wheelbase;


a = min(max(a_cmd, cfg.veh.a_emergency), cfg.veh.a_max);

d_target = min(max(delta_cmd, -cfg.veh.delta_max), cfg.veh.delta_max);
d_step   = cfg.veh.delta_rate * dt;
delta    = ego.delta + min(max(d_target - ego.delta, -d_step), d_step);
delta    = min(max(delta, -cfg.veh.delta_max), cfg.veh.delta_max);






if nargin < 6 || isempty(gear), gear = 1; end
v0 = ego.v;
if gear >= 0
    if v0 + a*dt < 0
        a = -v0 / dt;
    end
else
    if v0 + a*dt > 0
        a = -v0 / dt;
    elseif v0 + a*dt < -cfg.veh.v_reverse_max
        a = (-cfg.veh.v_reverse_max - v0) / dt;
    end
end





tanL = tan(delta) / L;

z0 = [ego.x; ego.y; ego.psi];
k1 = local_deriv(z0,                  0,     v0, a, tanL);
k2 = local_deriv(z0 + (dt/2) * k1,    dt/2,  v0, a, tanL);
k3 = local_deriv(z0 + (dt/2) * k2,    dt/2,  v0, a, tanL);
k4 = local_deriv(z0 +  dt    * k3,    dt,    v0, a, tanL);
z  = z0 + (dt/6) * (k1 + 2*k2 + 2*k3 + k4);

ego.x     = z(1);
ego.y     = z(2);
ego.psi   = ap_wrap_pi(z(3));
if gear >= 0
    ego.v = max(0, v0 + a*dt);
else
    ego.v = min(0, v0 + a*dt);
end
ego.delta = delta;
ego.a     = a;
end


function dz = local_deriv(z, tau, v0, a, tanL)

v  = v0 + a * tau;
dz = [v * cos(z(3));
      v * sin(z(3));
      v * tanL];
end
