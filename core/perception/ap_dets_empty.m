function d = ap_dets_empty()












d.x = zeros(0,1); d.y = zeros(0,1); d.R = zeros(2,2,0);
d.class = cell(0,1); d.sensor = cell(0,1); d.truth_id = zeros(0,1);
d.box = zeros(0,3); d.box_full = false(0,1); d.origin = zeros(0,2); d.road_psi = zeros(0,1);
d.surface = false(0,1);
end
