function ok = ap_test_all(filter)









if nargin < 1, filter = ''; end

tests = { ...
    'test_poly', ...
    'test_refpath', ...
    'test_frenet', ...
    'test_bicycle', ...
    'test_hungarian', ...
    'test_ekf', ...
    'test_collision', ...
    'test_perception', ...
    'test_footprint', ...
    'test_scenario_random', ...
    'test_scenario_closed_loop'};

if ~isempty(filter)
    keep = false(size(tests));
    for k = 1:numel(tests)
        keep(k) = ~isempty(strfind(tests{k}, filter));
    end
    tests = tests(keep);
end

fprintf('\n=== AP 26037 test suite ===\n');
n_pass = 0;
n_fail = 0;
n_skip = 0;
failures = {};

for k = 1:numel(tests)
    name = tests{k};
    if exist(name, 'file') ~= 2
        fprintf('  SKIP  %-28s (not implemented yet)\n', name);
        n_skip = n_skip + 1;
        continue;
    end
    t0 = tic;
    try
        feval(name);
        fprintf('  PASS  %-28s (%6.1f ms)\n', name, toc(t0)*1000);
        n_pass = n_pass + 1;
    catch err
        fprintf('  FAIL  %-28s %s\n', name, err.message);
        n_fail = n_fail + 1;
        failures{end+1} = sprintf('%s: %s', name, err.message);
    end
end

fprintf('--- %d passed, %d failed, %d skipped ---\n', n_pass, n_fail, n_skip);
if n_fail > 0
    fprintf('\nFailures:\n');
    for k = 1:numel(failures)
        fprintf('  * %s\n', failures{k});
    end
end
fprintf('\n');

ok = (n_fail == 0);
end
