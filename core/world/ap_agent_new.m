function a = ap_agent_new(id, class_name, x, y, psi, v_des, mode, wp, t_spawn)














if nargin < 8, wp = []; end
if nargin < 9 || isempty(t_spawn), t_spawn = 0; end

a.id      = id;
a.class   = lower(class_name);
a.props   = ap_agent_props(class_name);
a.x       = x;
a.y       = y;
a.psi     = psi;
a.v       = v_des;
a.v_des   = v_des;
a.mode    = mode;
a.wp      = wp;
a.wp_i    = 1;
a.t_spawn = t_spawn;
a.active  = true;
a.done    = false;
a.pass_psi = NaN;
a.pass_dir = NaN;
a.pass_t  = 0;
end
