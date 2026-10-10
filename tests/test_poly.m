function test_poly()







tol = 1e-9;


x0 = -1.3; v0 = 2.1; a0 = 0.4;
x1 =  2.7; v1 = -0.8; a1 = -0.25;
T  = 3.2;

c = ap_quintic(x0, v0, a0, x1, v1, a1, T);
ap_assert_true(numel(c) == 6, 'quintic must return 6 coefficients, got %d', numel(c));

[p, v, a] = ap_polyder_eval(c, [0; T]);
ap_assert_close(p(1), x0, tol, 'quintic x(0)');
ap_assert_close(v(1), v0, tol, 'quintic v(0)');
ap_assert_close(a(1), a0, tol, 'quintic a(0)');
ap_assert_close(p(2), x1, tol, 'quintic x(T)');
ap_assert_close(v(2), v1, tol, 'quintic v(T)');
ap_assert_close(a(2), a1, tol, 'quintic a(T)');


c = ap_quartic(x0, v0, a0, v1, a1, T);
ap_assert_true(numel(c) == 5, 'quartic must return 5 coefficients, got %d', numel(c));

[p, v, a] = ap_polyder_eval(c, [0; T]);
ap_assert_close(p(1), x0, tol, 'quartic x(0)');
ap_assert_close(v(1), v0, tol, 'quartic v(0)');
ap_assert_close(a(1), a0, tol, 'quartic a(0)');
ap_assert_close(v(2), v1, tol, 'quartic v(T)');
ap_assert_close(a(2), a1, tol, 'quartic a(T)');




h  = 1e-6;
t0 = 1.1;
[pm, vm, am] = ap_polyder_eval(c, t0 - h);
[pp, vp, ap] = ap_polyder_eval(c, t0 + h);
[~,  vc, ac, jc] = ap_polyder_eval(c, t0);

ap_assert_close((pp - pm) / (2*h), vc, 1e-5, 'dp/dt == v');
ap_assert_close((vp - vm) / (2*h), ac, 1e-5, 'dv/dt == a');
ap_assert_close((ap - am) / (2*h), jc, 1e-4, 'da/dt == j');
end
