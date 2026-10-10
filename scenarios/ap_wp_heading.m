function psi = ap_wp_heading(rp, s)

s   = min(max(s, 0), rp.length);
psi = ap_wrap_pi(interp1(rp.s, rp.psi, s, 'linear'));
end
