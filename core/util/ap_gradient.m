function g = ap_gradient(v, h)














if nargin < 2 || isempty(h)
    h = 1;
end

v = v(:);
n = numel(v);

g = zeros(n, 1);
if n == 1
    return;
end
if n == 2
    g(:) = (v(2) - v(1)) / h;
    return;
end

g(1)     = (v(2) - v(1)) / h;
g(2:n-1) = (v(3:n) - v(1:n-2)) / (2*h);
g(n)     = (v(n) - v(n-1)) / h;
end
