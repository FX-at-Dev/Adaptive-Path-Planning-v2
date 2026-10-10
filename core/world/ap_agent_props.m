function p = ap_agent_props(class_name)























persistent cache
key = lower(class_name);
if ~isempty(cache) && isfield(cache, key)
    p = cache.(key);
    return;
end

switch key
    case 'car'
        p = local_make(4.20, 1.80,  8.0, 16.0, 3.5, 0.7, 0.10, 0.15, 0.60);
    case 'bus'
        p = local_make(11.00, 2.60, 7.0, 14.0, 2.5, 0.4, 0.05, 0.10, 0.30);
    case 'truck'
        p = local_make(8.50, 2.50,  6.5, 13.0, 2.5, 0.4, 0.05, 0.10, 0.30);
    case 'auto'

        p = local_make(2.60, 1.40,  6.0, 12.0, 3.0, 1.6, 0.35, 0.60, 0.35);
    case 'two_wheeler'
        p = local_make(1.90, 0.70,  7.0, 18.0, 4.0, 2.2, 0.45, 0.70, 0.25);
    case 'bicycle'
        p = local_make(1.70, 0.60,  3.5,  7.0, 2.5, 1.2, 0.40, 0.45, 0.45);
    case 'pedestrian'
        p = local_make(0.60, 0.60,  1.3,  3.0, 2.0, 1.1, 1.20, 0.80, 0.50);
    case 'cattle'


        p = local_make(2.20, 0.90,  1.0,  3.0, 1.5, 0.9, 1.00, 0.95, 0.05);
    case 'pushcart'
        p = local_make(2.00, 1.20,  1.2,  2.5, 1.5, 0.5, 0.30, 0.35, 0.40);
    case 'static'

        p = local_make(3.00, 1.60,  0.0,  0.0, 0.0, 0.0, 0.00, 0.00, 0.00);
    otherwise
        error('ap_agent_props:unknownClass', ...
              'Unknown road-user class "%s".', class_name);
end
p.class = key;
if isempty(cache), cache = struct(); end
cache.(key) = p;
end


function p = local_make(len, wid, v_typ, v_max, a_brake, lat_agility, ...
                        heading_noise, erratic, yields)
p.length        = len;
p.width         = wid;
p.v_typ         = v_typ;
p.v_max         = v_max;
p.a_brake       = a_brake;
p.lat_agility   = lat_agility;
p.heading_noise = heading_noise;
p.erratic       = erratic;
p.yields        = yields;





p.n_discs = max(1, round(len / max(wid, 0.3)));
p.radius  = sqrt((len / p.n_discs / 2)^2 + (wid / 2)^2);
end
