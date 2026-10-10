function [x, P, nis, valid] = ap_ekf_update(x, P, z, R, gate_chi2)














if nargin < 5 || isempty(gate_chi2)
    gate_chi2 = Inf;
end

H = [1 0 0 0 0;
     0 1 0 0 0];

y = z(:) - H * x;
S = H * P * H.' + R;
S = 0.5 * (S + S.');

nis = y.' * (S \ y);

if nis > gate_chi2
    valid = false;
    return;
end
valid = true;

K = (P * H.') / S;

x = x + K * y;
x(4) = ap_wrap_pi(x(4));




I  = eye(5);
IKH = I - K * H;
P = IKH * P * IKH.' + K * R * K.';
P = 0.5 * (P + P.');
end
