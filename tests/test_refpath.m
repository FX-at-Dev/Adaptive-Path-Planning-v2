function test_refpath()







wp = [0 0; 50 0];
rp = ap_ref_path(wp, 0.25, 3.0);

ap_assert_close(rp.length, 50, 1e-9, 'straight path length');
ap_assert_close(rp.psi, zeros(size(rp.psi)), 1e-9, 'straight path heading');
ap_assert_close(rp.kappa, zeros(size(rp.kappa)), 1e-9, 'straight path curvature');
ap_assert_close(diff(rp.s), rp.ds * ones(numel(rp.s)-1, 1), 1e-9, 'uniform ds');




Rc  = 20;
th  = linspace(0, pi, 40)';
wp  = [Rc*sin(th), Rc*(1 - cos(th))];
rp  = ap_ref_path(wp, 0.20, 3.0);

ap_assert_close(rp.length, Rc*pi, 0.05, 'arc length of half circle');



m = 8:(numel(rp.kappa) - 8);
ap_assert_close(rp.kappa(m), (1/Rc) * ones(numel(m), 1), 2e-3, 'arc curvature');


ap_assert_close(rp.psi(1), 0, 1e-2, 'arc initial heading');
ap_assert_close(rp.psi(end), pi, 2e-2, 'arc final heading');
ap_assert_true(all(diff(rp.psi) > -1e-6), 'unwrapped heading must be monotonic on a CCW arc');


wp = [Rc*sin(th), -Rc*(1 - cos(th))];
rp = ap_ref_path(wp, 0.20, 3.0);
m  = 8:(numel(rp.kappa) - 8);
ap_assert_close(rp.kappa(m), (-1/Rc) * ones(numel(m), 1), 2e-3, 'CW arc curvature sign');
end
