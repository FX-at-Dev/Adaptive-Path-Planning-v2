function blocked = ap_ray_blocked(ex, ey, tx, ty, bx, by, br)











blocked = false;
if isempty(bx)
    return;
end

dx = tx - ex;
dy = ty - ey;
seg_len2 = dx*dx + dy*dy;
if seg_len2 < 1e-12
    return;
end

bx = bx(:); by = by(:); br = br(:);


tpar = ((bx - ex) * dx + (by - ey) * dy) / seg_len2;



cand = tpar > 0.02 & tpar < 0.98;
if ~any(cand)
    return;
end

tp = tpar(cand);
cxp = ex + tp * dx;
cyp = ey + tp * dy;

perp = sqrt((bx(cand) - cxp).^2 + (by(cand) - cyp).^2);
blocked = any(perp < br(cand));
end
