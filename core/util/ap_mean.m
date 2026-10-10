function m = ap_mean(x, dim)













persistent octave
if isempty(octave)
    octave = ap_is_octave();
end

if nargin < 2
    dim = find(size(x) ~= 1, 1);
    if isempty(dim)
        dim = 1;
    end
end

if octave
    if isempty(x) && ndims(x) == 2 && nargin < 2 && all(size(x) == 0)
        m = NaN;
        return;
    end
    m = sum(x, dim, 'extra') ./ size(x, dim);
else
    m = mean(x, dim);
end
end
