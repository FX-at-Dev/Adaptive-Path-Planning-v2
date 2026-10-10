function [s_start, t_wait] = ap_upstream(rp, s0, v, t_due, dirn)









s_start = s0 - dirn * v * t_due;
s_clamped = min(max(s_start, 0), rp.length);
t_wait = abs(s_clamped - s_start) / max(v, 0.1);
s_start = s_clamped;
end
