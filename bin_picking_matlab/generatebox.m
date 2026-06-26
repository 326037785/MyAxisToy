function box = generatebox(size_spec)
%GENERATEBOX Create a box-shaped item with geometry and pose fields.
%
% Supported size_spec forms:
%   scalar M       : each dimension is a random integer in [1, M]
%   [w h d]        : fixed dimensions
%   [wMin hMin dMin;
%    wMax hMax dMax] : independently sampled integer dimensions
%
% The returned structure is also compatible with simulate_bin_picking.m.

    validateattributes(size_spec, {'numeric'}, {'real', 'finite', 'positive'});

    if isscalar(size_spec)
        dims = randi([1, floor(size_spec)], 1, 3);
    elseif isequal(size(size_spec), [1, 3])
        dims = double(size_spec);
    elseif isequal(size(size_spec), [2, 3])
        lo = ceil(size_spec(1, :));
        hi = floor(size_spec(2, :));
        if any(lo > hi)
            error('generatebox:InvalidRange', ...
                'Each minimum dimension must be no greater than its maximum.');
        end
        dims = arrayfun(@(a, b) randi([a, b]), lo, hi);
    else
        error('generatebox:InvalidSizeSpec', ...
            'size_spec must be scalar, 1x3 fixed dimensions, or a 2x3 range.');
    end

    box.w = dims(1);
    box.h = dims(2);
    box.d = dims(3);
    box.size = dims;

    box.position = [0, 0, 0];
    box.R = eye(3);
    box.orientation = [0, 0, 0, 1];  % quaternion [x y z w]

    % Fields populated by the packing script.
    box.footprint = zeros(0, 2);
    box.z_bottom = 0;
    box.z_top = 0;
    box.color = [0.2, 0.5, 0.9];
    box.support_ids = zeros(1, 0);
end
