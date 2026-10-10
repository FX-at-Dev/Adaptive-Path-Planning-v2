function [s, d, kappa_ref, psi_ref] = ap_cart2frenet(rp, x, y)











dx = rp.x - x;
dy = rp.y - y;
[~, i] = min(dx.^2 + dy.^2);

ps = rp.psi(i);
ct = cos(ps);
st = sin(ps);


ex = x - rp.x(i);
ey = y - rp.y(i);


s_off = ct * ex + st * ey;
d     = -st * ex + ct * ey;

s = rp.s(i) + s_off;
s = min(max(s, 0), rp.length);

kappa_ref = rp.kappa(i);
psi_ref   = ps;
end
