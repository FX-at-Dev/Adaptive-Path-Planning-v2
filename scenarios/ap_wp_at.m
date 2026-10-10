function [x, y] = ap_wp_at(rp, s, d)









s = min(max(s, 0), rp.length);
[x, y] = ap_frenet2cart(rp, s, d);
end
