function c = ap_quartic(x0, v0, a0, v1, a1, T)










c0 = x0;
c1 = v0;
c2 = 0.5 * a0;

T2 = T*T;  T3 = T2*T;

A = [3*T2,   4*T3;
      6*T,  12*T2];

b = [v1 - (c1 + 2*c2*T);
     a1 - 2*c2];

c34 = A \ b;
c = [c0, c1, c2, c34(1), c34(2)];
end
