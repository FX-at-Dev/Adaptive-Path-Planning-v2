function ap_rng(seed)





if ap_is_octave()
    rand('state', seed);
    randn('state', seed);
else
    rng(seed, 'twister');
end
end
