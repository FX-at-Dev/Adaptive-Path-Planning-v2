function test_hungarian()







ap_rng(12345);


for n = 1:6
    for trial = 1:12
        C = round(rand(n, n) * 20);
        [assign, total] = ap_hungarian(C);

        ap_assert_true(numel(assign) == n, 'assign length for %dx%d', n, n);
        ap_assert_true(numel(unique(assign)) == n, 'assignment must be a permutation');
        ap_assert_true(all(assign >= 1 & assign <= n), 'assignment in range');

        best = local_brute_force(C);
        ap_assert_close(total, best, 1e-9, sprintf('optimal cost n=%d', n));


        chk = 0;
        for r = 1:n
            chk = chk + C(r, assign(r));
        end
        ap_assert_close(total, chk, 1e-9, 'total matches assignment');
    end
end


for trial = 1:12
    n = 3; m = 6;
    C = round(rand(n, m) * 20);
    [assign, total] = ap_hungarian(C);
    ap_assert_true(numel(assign) == n, 'wide: one entry per row');
    ap_assert_true(all(assign >= 1), 'wide: every row matched');
    ap_assert_true(numel(unique(assign)) == n, 'wide: columns used at most once');
    ap_assert_close(total, local_brute_force(C), 1e-9, 'wide optimal cost');
end


for trial = 1:12
    n = 6; m = 3;
    C = round(rand(n, m) * 20);
    [assign, total] = ap_hungarian(C);
    ap_assert_true(numel(assign) == n, 'tall: one entry per row');
    ap_assert_true(sum(assign > 0) == m, 'tall: exactly m rows matched');
    matched = assign(assign > 0);
    ap_assert_true(numel(unique(matched)) == m, 'tall: columns used once');
    ap_assert_close(total, local_brute_force(C), 1e-9, 'tall optimal cost');
end


[a, t] = ap_hungarian(zeros(0, 4));
ap_assert_true(isempty(a) && t == 0, 'empty rows handled');
[a, t] = ap_hungarian(zeros(3, 0));
ap_assert_true(numel(a) == 3 && all(a == 0) && t == 0, 'empty columns handled');







C = [1 2; 3 100];
[assign, total] = ap_hungarian(C);
ap_assert_close(total, 5, 1e-12, 'greedy trap total');
ap_assert_true(assign(1) == 2 && assign(2) == 1, 'greedy trap assignment');
end


function best = local_brute_force(C)



[n, m] = size(C);
if n > m
    C = C.';
    [n, m] = size(C);
end
cols = perms(1:m);
best = Inf;
for r = 1:size(cols, 1)
    pick = cols(r, 1:n);
    c    = sum(C(sub2ind(size(C), (1:n).', pick(:))));
    best = min(best, c);
end
end
