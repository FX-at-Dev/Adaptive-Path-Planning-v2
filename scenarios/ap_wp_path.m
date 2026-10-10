function wp = ap_wp_path(rp, s_list, d_list)




if numel(s_list) ~= numel(d_list)
    error('ap_wp_path:size', 's_list and d_list must be the same length.');
end
wp = zeros(numel(s_list), 2);
for k = 1:numel(s_list)
    [wp(k,1), wp(k,2)] = ap_wp_at(rp, s_list(k), d_list(k));
end
end
