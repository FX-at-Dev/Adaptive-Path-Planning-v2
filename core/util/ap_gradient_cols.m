function G = ap_gradient_cols(M, h)












if nargin < 2 || isempty(h)
    h = 1;
end

[n, ~] = size(M);
G = zeros(size(M));

if n == 1
    return;
end
if n == 2
    d = (M(2,:) - M(1,:)) / h;
    G(1,:) = d;
    G(2,:) = d;
    return;
end

G(1,:)     = (M(2,:) - M(1,:)) / h;
G(2:n-1,:) = (M(3:n,:) - M(1:n-2,:)) / (2*h);
G(n,:)     = (M(n,:) - M(n-1,:)) / h;
end
