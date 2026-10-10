function c = ap_quintic(x0, v0, a0, x1, v1, a1, T)











c0 = x0;
c1 = v0;
c2 = 0.5 * a0;


T2 = T*T;  T3 = T2*T;  T4 = T3*T;  T5 = T4*T;

A = [    T3,     T4,      T5;
     3*T2,   4*T3,    5*T4;
      6*T,  12*T2,   20*T3];

b = [x1 - (c0 + c1*T + c2*T2);
     v1 - (c1 + 2*c2*T);
     a1 - 2*c2];

c345 = A \ b;
c = [c0, c1, c2, c345(1), c345(2), c345(3)];
end
