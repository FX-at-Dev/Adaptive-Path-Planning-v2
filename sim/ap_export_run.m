function path_out = ap_export_run(scn, cfg, result, log, out_dir)



















if nargin < 5 || isempty(out_dir)
    out_dir = fullfile(pwd, 'results');
end
if exist(out_dir, 'dir') ~= 7
    mkdir(out_dir);
end

m = ap_metrics(result, log, cfg);




classes = {};
for k = 1:numel(log.snaps)
    classes = [classes, {log.snaps{k}.agents.class}];
end
think = ~isempty(log.snaps) && isfield(log.snaps{1}, 'think');
data = ap_export_header(scn, cfg, classes, think);


data.series.t          = log.t.';
data.series.x          = log.x.';
data.series.y          = log.y.';
data.series.psi        = log.psi.';
data.series.v          = log.v.';
data.series.a          = log.a.';
data.series.delta      = log.delta.';
data.series.min_clear  = log.min_clear.';
data.series.v_cap      = log.v_cap.';
data.series.plan_risk  = log.plan_risk.';
data.series.feasible   = double(log.feasible).';
data.series.n_tracks   = log.n_confirmed.';
data.series.state      = log.state(:).';



li = find(isfinite(log.latency_ms));
data.series.latency_t  = log.t(li).';
data.series.latency_ms = log.latency_ms(li).';


frames = cell(1, numel(log.snaps));
for k = 1:numel(log.snaps)
    frames{k} = ap_export_frame(log.snaps{k});
end
data.frames = frames;


data.metrics = m;
data.result.reached   = double(result.reached);
data.result.collided  = double(result.collided);
data.result.t_reached = result.t_reached;
data.result.t_end     = result.t_end;
data.result.min_clear = result.min_clear;


path_out = fullfile(out_dir, [scn.name '.json']);

if exist('jsonencode', 'builtin') == 5 || exist('jsonencode', 'file') == 2
    txt = jsonencode(data);
else
    error('ap_export_run:noJson', ...
          ['jsonencode is unavailable on this interpreter. It ships with ' ...
           'MATLAB R2016b+ and Octave 7+; upgrade, or render from MATLAB ' ...
           'with ap_visualize instead.']);
end

fid = fopen(path_out, 'w');
if fid < 0
    error('ap_export_run:write', 'Could not open %s for writing.', path_out);
end
fwrite(fid, txt, 'char');
fclose(fid);

fprintf('  exported %s (%.1f KB, %d frames)\n', path_out, numel(txt)/1024, numel(frames));
end
