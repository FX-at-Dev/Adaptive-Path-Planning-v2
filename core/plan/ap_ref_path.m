function rp = ap_ref_path(wp, ds, halfwidth)


















if nargin < 2 || isempty(ds),        ds = 0.25;  end
if nargin < 3 || isempty(halfwidth), halfwidth = 3.0; end

wp = double(wp);
if size(wp, 2) ~= 2 || size(wp, 1) < 2
    error('ap_ref_path:badInput', 'wp must be Nx2 with at least 2 rows.');
end


keep = [true; sum(diff(wp, 1, 1).^2, 2) > 1e-12];
wp   = wp(keep, :);
n    = size(wp, 1);
if n < 2
    error('ap_ref_path:degenerate', 'wp collapsed to a single point.');
end


seg    = sqrt(sum(diff(wp, 1, 1).^2, 2));
t_raw  = [0; cumsum(seg)];
L_est  = t_raw(end);

n_dense = max(400, ceil(4 * L_est / ds));
tq      = linspace(0, L_est, n_dense)';

if n >= 3
    method = 'spline';   % curvature-continuous through interior waypoints
else
    method = 'linear';   % two points define a straight line; spline is moot
end
xd = interp1(t_raw, wp(:,1), tq, method);
yd = interp1(t_raw, wp(:,2), tq, method);


sd = [0; cumsum(sqrt(diff(xd).^2 + diff(yd).^2))];
L  = sd(end);






n_s = max(3, ceil(L / ds) + 1);
s   = linspace(0, L, n_s)';
ds  = s(2) - s(1);



[sd_u, iu] = unique(sd);
x = interp1(sd_u, xd(iu), s, 'linear');
y = interp1(sd_u, yd(iu), s, 'linear');



dx  = gradient(x, ds);
dy  = gradient(y, ds);
ddx = gradient(dx, ds);
ddy = gradient(dy, ds);

psi = atan2(dy, dx);
psi = unwrap(psi);

denom = (dx.^2 + dy.^2).^1.5;
denom(denom < 1e-12) = 1e-12;
kappa = (dx .* ddy - dy .* ddx) ./ denom;

rp.s         = s;
rp.x         = x;
rp.y         = y;
rp.psi       = psi;
rp.kappa     = kappa;
rp.ds        = ds;
rp.length    = L;
if isscalar(halfwidth)
    rp.halfwidth = halfwidth * ones(size(s));
else
    rp.halfwidth = interp1(linspace(0, L, numel(halfwidth))', halfwidth(:), s, 'linear');
end
end
