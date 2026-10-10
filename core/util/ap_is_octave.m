function tf = ap_is_octave()



persistent cached
if isempty(cached)
    cached = (exist('OCTAVE_VERSION', 'builtin') ~= 0);
end
tf = cached;
end
