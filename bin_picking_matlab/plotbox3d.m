function h_patch = plotbox3d(box_struct, position, color)
%PLOTBOX3D Render an oriented 3-D box.
%
%   plotbox3d(box_struct)
%   plotbox3d(box_struct, position)
%   plotbox3d(box_struct, position, color)
%
% box_struct may contain:
%   w, h, d       dimensions
%   position      center position [x y z]
%   R             3x3 local-to-world rotation matrix
%   orientation   quaternion [x y z w], used only if R is absent

    if nargin < 2 || isempty(position)
        if isfield(box_struct, 'position')
            position = box_struct.position;
        else
            position = [0, 0, 0];
        end
    end

    if nargin < 3 || isempty(color)
        if isfield(box_struct, 'color')
            color = box_struct.color;
        else
            color = [0.2, 0.5, 0.9];
        end
    end

    validateattributes(position, {'numeric'}, {'vector', 'numel', 3, 'finite'});
    position = reshape(position, 1, 3);

    if isfield(box_struct, 'R') && isequal(size(box_struct.R), [3, 3])
        R = box_struct.R;
    elseif isfield(box_struct, 'orientation') && numel(box_struct.orientation) == 4
        R = quaternionToRotation(box_struct.orientation);
    else
        R = eye(3);
    end

    half_size = 0.5 * [box_struct.w, box_struct.h, box_struct.d];
    dx = half_size(1);
    dy = half_size(2);
    dz = half_size(3);

    local_vertices = [
        -dx -dy -dz;
         dx -dy -dz;
         dx  dy -dz;
        -dx  dy -dz;
        -dx -dy  dz;
         dx -dy  dz;
         dx  dy  dz;
        -dx  dy  dz
    ];

    % Row-vector representation: p_world = p_local * R' + translation.
    vertices = local_vertices * R.' + position;

    faces = [
        1 4 3 2;  % bottom
        5 6 7 8;  % top
        1 2 6 5;
        2 3 7 6;
        3 4 8 7;
        4 1 5 8
    ];

    hold on;
    h_patch = patch( ...
        'Vertices', vertices, ...
        'Faces', faces, ...
        'FaceColor', color, ...
        'FaceAlpha', 0.78, ...
        'EdgeColor', [0.12, 0.12, 0.12], ...
        'LineWidth', 0.8);
end

function R = quaternionToRotation(q)
% q = [x y z w]
    q = double(q(:).');
    q = q / norm(q);
    x = q(1); y = q(2); z = q(3); w = q(4);

    R = [
        1 - 2*(y*y + z*z), 2*(x*y - z*w),     2*(x*z + y*w);
        2*(x*y + z*w),     1 - 2*(x*x + z*z), 2*(y*z - x*w);
        2*(x*z - y*w),     2*(y*z + x*w),     1 - 2*(x*x + y*y)
    ];
end
