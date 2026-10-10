function [x, y, psi] = ap_frenet2cart(rp, s, d, dprime)
















s = s(:);
d = d(:);
if nargin < 4 || isempty(dprime)
    dprime = zeros(size(s));
else
    dprime = dprime(:);
end

N  = numel(rp.s);
ds = rp.ds;

sc = min(max(s, 0), rp.length);



i0 = floor(sc / ds) + 1;
i0 = min(max(i0, 1), N - 1);
i1 = i0 + 1;
w  = (sc - (i0 - 1) * ds) / ds;

rx  = rp.x(i0)     .* (1 - w) + rp.x(i1)     .* w;
ry  = rp.y(i0)     .* (1 - w) + rp.y(i1)     .* w;
rps = rp.psi(i0)   .* (1 - w) + rp.psi(i1)   .* w;
rk  = rp.kappa(i0) .* (1 - w) + rp.kappa(i1) .* w;


x = rx - d .* sin(rps);
y = ry + d .* cos(rps);




one_minus_kd = 1 - rk .* d;
one_minus_kd(abs(one_minus_kd) < 1e-6) = 1e-6;
psi = ap_wrap_pi(rps + atan2(dprime, one_minus_kd));
end
