function [p, v, a, j] = ap_polyder_eval(c, t)










t = t(:);
n = numel(c);

p = zeros(size(t));
v = zeros(size(t));
a = zeros(size(t));
j = zeros(size(t));

for k = 1:n
    e = k - 1;
    p = p + c(k) * t.^e;
    if e >= 1
        v = v + c(k) * e * t.^(e-1);
    end
    if e >= 2
        a = a + c(k) * e*(e-1) * t.^(e-2);
    end
    if e >= 3
        j = j + c(k) * e*(e-1)*(e-2) * t.^(e-3);
    end
end
end
