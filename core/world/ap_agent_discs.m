function [cx, cy] = ap_agent_discs(x, y, psi, props)










n   = props.n_discs;
seg = props.length / n;


offs = -props.length/2 + seg * ((1:n) - 0.5);

x = x(:); y = y(:); psi = psi(:);
cx = x * ones(1, n) + cos(psi) * offs;
cy = y * ones(1, n) + sin(psi) * offs;
end
