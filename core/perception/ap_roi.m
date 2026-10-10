function z = ap_roi(ctx, x, y)





gx = floor((x - ctx.x0) / ctx.cell) + 1;
gy = floor((y - ctx.y0) / ctx.cell) + 1;
in = gx >= 1 & gx <= ctx.nx & gy >= 1 & gy <= ctx.ny;
z = zeros(size(x));
z(in) = double(ctx.roi(sub2ind([ctx.ny, ctx.nx], gy(in), gx(in))));
end
