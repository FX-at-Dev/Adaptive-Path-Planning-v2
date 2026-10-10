function test_frenet()







rp = ap_ref_path([0 0; 60 0], 0.25, 3.0);

s_in = [5; 20; 41.7];
d_in = [0; 2.4; -1.9];
[x, y] = ap_frenet2cart(rp, s_in, d_in);


ap_assert_close(y, d_in, 1e-9, 'straight: +d is to the left');
ap_assert_close(x, s_in, 1e-9, 'straight: s maps to x');

for k = 1:numel(s_in)
    [s_out, d_out] = ap_cart2frenet(rp, x(k), y(k));
    ap_assert_close(s_out, s_in(k), 1e-6, 'straight round-trip s');
    ap_assert_close(d_out, d_in(k), 1e-6, 'straight round-trip d');
end


Rc = 25;
th = linspace(0, pi/2, 30)';
rp = ap_ref_path([Rc*sin(th), Rc*(1 - cos(th))], 0.20, 3.0);

s_in = [3; 12; 25; 35];
d_in = [0; 1.5; -2.2; 0.8];
[x, y] = ap_frenet2cart(rp, s_in, d_in);










for k = 1:numel(s_in)
    [s_out, d_out] = ap_cart2frenet(rp, x(k), y(k));
    ap_assert_close(s_out, s_in(k), 1.5e-2, 'curved round-trip s');
    ap_assert_close(d_out, d_in(k), 1.0e-3, 'curved round-trip d');
end




err_coarse = local_max_d_err(Rc, th, 0.40);
err_fine   = local_max_d_err(Rc, th, 0.05);
ap_assert_true(err_fine < err_coarse / 10, ...
    'lateral error must converge with sampling: %.3e at ds=0.40 vs %.3e at ds=0.05', ...
    err_coarse, err_fine);




[~, ~, psi_off] = ap_frenet2cart(rp, 20, 2.0, 0);
[~, ~, ~, psi_ref] = ap_cart2frenet(rp, ...
    interp1(rp.s, rp.x, 20) - 2.0*sin(interp1(rp.s, rp.psi, 20)), ...
    interp1(rp.s, rp.y, 20) + 2.0*cos(interp1(rp.s, rp.psi, 20)));
ap_assert_close(ap_wrap_pi(psi_off - psi_ref), 0, 5e-3, 'parallel offset heading');
end


function e = local_max_d_err(Rc, th, ds)

rp = ap_ref_path([Rc*sin(th), Rc*(1 - cos(th))], ds, 3.0);
s_in = [3; 12; 25; 35];
d_in = [0; 1.5; -2.2; 0.8];
[x, y] = ap_frenet2cart(rp, s_in, d_in);
e = 0;
for k = 1:numel(s_in)
    [~, d_out] = ap_cart2frenet(rp, x(k), y(k));
    e = max(e, abs(d_out - d_in(k)));
end
end
