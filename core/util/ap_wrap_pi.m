function a = ap_wrap_pi(a)

a = mod(a + pi, 2*pi) - pi;


a(a == -pi) = pi;
end
