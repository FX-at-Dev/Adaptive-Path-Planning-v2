function [assign, total] = ap_hungarian(C)















[n, m] = size(C);
if n == 0 || m == 0
    assign = zeros(n, 1);
    total  = 0;
    return;
end



transposed = false;
if n > m
    C = C.';
    [n, m] = size(C);
    transposed = true;
end

BIG = 1e18;

u   = zeros(n+1, 1);
v   = zeros(m+1, 1);
p   = zeros(m+1, 1);
way = zeros(m+1, 1);

for i = 1:n
    p(1) = i;
    j0   = 0;
    minv = BIG * ones(m+1, 1);
    used = false(m+1, 1);


    while true
        used(j0+1) = true;
        i0 = p(j0+1);

        idx = find(~used(2:m+1));
        cur = C(i0, idx).' - u(i0+1) - v(idx+1);

        better = cur < minv(idx+1);
        minv(idx(better)+1) = cur(better);
        way(idx(better)+1)  = j0;

        [delta, k] = min(minv(idx+1));
        j1 = idx(k);




        uidx = find(used);
        u(p(uidx)+1) = u(p(uidx)+1) + delta;
        v(uidx)      = v(uidx) - delta;

        nidx = find(~used);
        minv(nidx) = minv(nidx) - delta;

        j0 = j1;
        if p(j0+1) == 0
            break;
        end
    end


    while true
        j1       = way(j0+1);
        p(j0+1)  = p(j1+1);
        j0       = j1;
        if j0 == 0
            break;
        end
    end
end

assign = zeros(n, 1);
for j = 1:m
    if p(j+1) > 0
        assign(p(j+1)) = j;
    end
end

if transposed

    inv_assign = zeros(m, 1);
    for r = 1:n
        if assign(r) > 0
            inv_assign(assign(r)) = r;
        end
    end
    assign = inv_assign;
end

total = 0;
if transposed
    C = C.';
end
for r = 1:numel(assign)
    if assign(r) > 0
        total = total + C(r, assign(r));
    end
end
end
