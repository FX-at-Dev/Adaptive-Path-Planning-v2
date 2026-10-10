function world = ap_world_step(world, dt, cfg)









t = world.t + dt;


[ecx, ecy, erad] = ap_ego_discs(world.ego.x, world.ego.y, world.ego.psi, cfg);

for k = 1:numel(world.agents)
    a = world.agents(k);

    if ~a.active
        continue;
    end
    if t < a.t_spawn


        a.v = 0;
        world.agents(k) = a;
        continue;
    end







    if strcmp(a.mode, 'static')
        a.v = 0;
        world.agents(k) = a;
        continue;
    end

    p = a.props;







    ex = world.ego.x - a.x;
    ey = world.ego.y - a.y;
    fwd = cos(a.psi) * ex + sin(a.psi) * ey;
    lat = -sin(a.psi) * ex + cos(a.psi) * ey;














    is_crosser = strcmp(p.class, 'pedestrian') || strcmp(p.class, 'cattle');

    d_react = 6.0 + 10.0 * p.yields;
    react   = 1.0;
    avoid   = 0;










    if is_crosser
        for q = 1:numel(ecx)
            qx = ecx(q) - a.x;
            qy = ecy(q) - a.y;
            qf = cos(a.psi) * qx + sin(a.psi) * qy;
            ql = -sin(a.psi) * qx + cos(a.psi) * qy;

            if qf > 0 && qf < erad + p.length / 2 + 0.6 && abs(ql) < erad + p.width / 2 + 0.4
                react = 0;
            end
        end
    end
    if is_crosser && fwd > 0 && fwd < 2.2 && abs(lat) < 1.6
        react = 0;
    end






    side_psi = NaN;
    if is_crosser && react == 0 && world.ego.v < 0.3
        c_now = local_clear_after(a, a.psi, 0, p, ecx, ecy, erad);
        need = max(0.3, min(c_now, 0.8));
        away = -1;
        if lat < 0, away = 1; end
        for turn = [0.5, 1.0, 1.5, 2.0]
            for sgn = [away, -away]
                h = a.psi + sgn * turn;
                if local_clear_after(a, h, min(a.v_des, 1.2), p, ecx, ecy, erad) >= need
                    side_psi = h;
                    break;
                end
            end
            if isfinite(side_psi), break; end
        end
    end

    if ~is_crosser && fwd > 0 && fwd < d_react && abs(lat) < 3.2
        strength = 1 - fwd / d_react;












        react = (fwd - 3.0) / max(d_react - 3.0, 1.0);
        react = min(max(react, 0), 1);











        if lat >= 0
            avoid = -1.1 * strength;
        else
            avoid =  1.1 * strength;
        end
    end









    if ~is_crosser
        for q = 1:numel(ecx)
            qx = ecx(q) - a.x;
            qy = ecy(q) - a.y;
            qf = cos(a.psi) * qx + sin(a.psi) * qy;
            ql = -sin(a.psi) * qx + cos(a.psi) * qy;
            if qf > 0 && qf < 40 && abs(ql) < erad + p.width / 2 + 0.3
                gap = qf - erad - p.length / 2 - 1.0;
                v_safe = sqrt(2 * 0.8 * p.a_brake * max(gap, 0));
                react = min(react, v_safe / max(a.v_des, 0.1));
            end
        end






        c_now = local_clear_after(a, a.psi, 0, p, ecx, ecy, erad);
        if isfinite(a.pass_psi)
            a.pass_t = a.pass_t + dt;
            behind = cos(a.pass_dir) * ex + sin(a.pass_dir) * ey < -(p.length / 2 + 1.0);
            if world.ego.v > 0.5 || (behind && c_now > 0.8) || a.pass_t > 10
                a.pass_psi = NaN;
            end
        elseif world.ego.v < 0.2 && fwd > 0 && a.v < 0.05 && c_now < 3.0
            v_c = 0.8;
            need = max(0.4, min(c_now, 1.0) - 0.02);
            if c_now < 0.4, need = c_now; end
            away = -1;
            if lat < 0, away = 1; end
            for turn = [0.3, 0.6, 0.9, 1.2, 1.5]
                for sgn = [away, -away]
                    h = a.psi + sgn * turn;
                    if local_clear_after(a, h, v_c, p, ecx, ecy, erad) >= need
                        a.pass_psi = h;
                        a.pass_dir = a.psi;
                        a.pass_t = 0;
                        break;
                    end
                end
                if isfinite(a.pass_psi), break; end
            end
        end
        if isfinite(a.pass_psi)
            side_psi = a.pass_psi;
            react = 0.8 / max(a.v_des, 0.1);
        end
    end


    if strcmp(a.mode, 'cross')
        psi_des = a.psi;
    else









        if isempty(a.wp) || a.wp_i > size(a.wp, 1)
            a.done  = true;
            psi_des = a.psi;
        else
            tgt = a.wp(a.wp_i, :);
            if hypot(tgt(1) - a.x, tgt(2) - a.y) < max(1.5, 0.6 * a.v)
                a.wp_i = a.wp_i + 1;
            end
            if a.wp_i > size(a.wp, 1)
                a.done  = true;
                psi_des = a.psi;
            else
                tgt = a.wp(a.wp_i, :);
                psi_des = atan2(tgt(2) - a.y, tgt(1) - a.x);
            end
        end
    end

    if isfinite(side_psi)
        psi_des = side_psi;
        avoid = 0;
    end






    if ~is_crosser && (avoid ~= 0 || isfinite(side_psi)) && a.v > 0.1
        h = psi_des + avoid;
        look = max(a.v, 1.0) * 1.0;
        [~, d1] = ap_cart2frenet(world.rp, a.x + look * cos(h), a.y + look * sin(h));
        [s0a, d0a] = ap_cart2frenet(world.rp, a.x, a.y);
        hw_a = interp1(world.rp.s, world.rp.halfwidth, min(max(s0a, 0), world.rp.length), 'linear');
        lim = hw_a - p.width / 2 + 0.1;
        if abs(d1) > lim && abs(d1) > abs(d0a)
            avoid = 0;
            if isfinite(side_psi), psi_des = a.psi; react = 0; end
        end
    end


    yaw_max = 2.4 / max(1.0, p.length / 2);
    dpsi    = ap_wrap_pi(psi_des + avoid - a.psi);
    a.psi   = a.psi + min(max(dpsi, -yaw_max*dt), yaw_max*dt);


    if p.erratic > 0
        psi_was = a.psi;
        a.psi = a.psi + p.heading_noise * p.erratic * sqrt(dt) * randn();




        [ox, oy] = ap_agent_discs(a.x, a.y, psi_was, p);
        [nx, ny] = ap_agent_discs(a.x, a.y, a.psi, p);
        d_was = min(min(hypot(ox(:) - ecx(:).', oy(:) - ecy(:).')));
        d_now = min(min(hypot(nx(:) - ecx(:).', ny(:) - ecy(:).')));
        if d_now < d_was && d_now < erad + p.radius + 0.3
            a.psi = psi_was;
        end
    end
    a.psi = ap_wrap_pi(a.psi);


    v_target = react * a.v_des * (1 + 0.05 * p.erratic * randn());
    a.v = a.v + min(max(v_target - a.v, -p.a_brake*dt), p.a_brake*dt);
    a.v = min(max(a.v, 0), p.v_max);





    nx = a.x + a.v * cos(a.psi) * dt;
    ny = a.y + a.v * sin(a.psi) * dt;
    if a.v > 0
        c_was = local_clear_at(a.x, a.y, a.psi, p, ecx, ecy, erad);
        c_new = local_clear_at(nx, ny, a.psi, p, ecx, ecy, erad);
        if c_new < c_was && c_new < 0.4
            nx = a.x; ny = a.y; a.v = 0;
        end
    end
    a.x = nx;
    a.y = ny;

    world.agents(k) = a;
end

world.t = t;
end


function c = local_clear_after(a, h, v, p, ecx, ecy, erad)

[nx, ny] = ap_agent_discs(a.x + v * cos(h), a.y + v * sin(h), h, p);
c = min(min(hypot(nx(:) - ecx(:).', ny(:) - ecy(:).'))) - erad - p.radius;
end


function c = local_clear_at(x, y, psi, p, ecx, ecy, erad)

[nx, ny] = ap_agent_discs(x, y, psi, p);
c = min(min(hypot(nx(:) - ecx(:).', ny(:) - ecy(:).'))) - erad - p.radius;
end
