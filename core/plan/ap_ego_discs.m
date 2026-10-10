function [cx, cy, r] = ap_ego_discs(x, y, psi, cfg)















n = cfg.ego.n_discs;
L = cfg.ego.length;
W = cfg.ego.width;



rear_overhang = 0.5 * (L - cfg.ego.wheelbase);
x_back  = -rear_overhang;
x_front =  L - rear_overhang;



seg = L / n;
r   = sqrt((seg/2)^2 + (W/2)^2);

x = x(:); y = y(:); psi = psi(:);
offs = x_back + seg * ((1:n) - 0.5);

c = cos(psi);
s = sin(psi);

cx = x * ones(1, n) + c * offs;
cy = y * ones(1, n) + s * offs;
end
