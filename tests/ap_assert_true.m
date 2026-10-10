function ap_assert_true(cond, msg, varargin)

if ~all(cond(:))
    error('ap:assertTrue', msg, varargin{:});
end
end
