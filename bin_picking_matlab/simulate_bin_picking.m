%% SIMULATE_BIN_PICKING
% Quasi-static generation of randomly arranged box-shaped items in an
% open-top bin. The model enforces:
%   1. no wall penetration;
%   2. no item-item penetration;
%   3. vertical gravitational settling;
%   4. static support under the projected center of mass;
%   5. a minimum contact-area ratio.
%
% This is a geometry-based static packing approximation, not a rigid-body
% dynamics simulation. It deliberately ignores deformation, sliding,
% bouncing, friction evolution, and impact forces.

clc;
clear;
close all;

%% ------------------------- hard-coded settings -------------------------
rng(12);                       % reproducible random scene
ITEM_COUNT = 90;               % requested number of items
BIN_SIZE = [70, 52, 42];       % [length(X), width(Y), height(Z)]

% Set true for mathematically exact cubes. Set false for rectangular boxes,
% which makes the random stable-face orientation visually meaningful.
USE_TRUE_CUBES = false;
CUBE_SIDE_RANGE = [6, 10];
CUBOID_SIZE_RANGE = [
    6, 5, 4;                   % minimum [w h d]
    12, 10, 8                  % maximum [w h d]
];

MAX_ATTEMPTS_PER_ITEM = 5000;
MIN_SUPPORT_RATIO = 0.22;      % contact area / bottom-face area
GEOMETRY_TOL = 1e-8;
ANIMATE_PLACEMENT = false;

%% ---------------------------- visualization ----------------------------
fig = figure( ...
    'Name', 'Quasi-static bin-picking scene', ...
    'Color', 'w');
ax = axes('Parent', fig);
hold(ax, 'on');
grid(ax, 'on');
axis(ax, 'equal');
view(ax, -38, 25);
xlabel(ax, 'X');
ylabel(ax, 'Y');
zlabel(ax, 'Z');
title(ax, 'Random supported box packing');

drawOpenBin(BIN_SIZE);
colors = lines(max(ITEM_COUNT, 7));

%% ----------------------------- placement -------------------------------
placed = repmat(generatebox([1, 1, 1]), 0, 1);

for item_id = 1:ITEM_COUNT
    if USE_TRUE_CUBES
        side = randi(CUBE_SIDE_RANGE);
        item = generatebox([side, side, side]);
    else
        item = generatebox(CUBOID_SIZE_RANGE);
    end

    item.color = colors(item_id, :);
    accepted = false;

    for attempt = 1:MAX_ATTEMPTS_PER_ITEM
        candidate = randomStablePose(item);
        [candidate, accepted] = settleCandidate( ...
            candidate, placed, BIN_SIZE, ...
            MIN_SUPPORT_RATIO, GEOMETRY_TOL);

        if accepted
            placed(end + 1, 1) = candidate; %#ok<SAGROW>

            if ANIMATE_PLACEMENT
                plotbox3d(candidate);
                drawnow;
            end
            break;
        end
    end

    if ~accepted
        warning('simulate_bin_picking:PlacementFailed', ...
            ['Only %d of %d items were placed. Increase the bin size, ', ...
             'reduce item size/count, lower MIN_SUPPORT_RATIO, or ', ...
             'increase MAX_ATTEMPTS_PER_ITEM.'], ...
            numel(placed), ITEM_COUNT);
        break;
    end
end

%% ----------------------------- final plot ------------------------------
if ~ANIMATE_PLACEMENT
    for i = 1:numel(placed)
        plotbox3d(placed(i));
    end
end

xlim(ax, 0.58 * BIN_SIZE(1) * [-1, 1]);
ylim(ax, 0.58 * BIN_SIZE(2) * [-1, 1]);
zlim(ax, [0, 1.08 * BIN_SIZE(3)]);
camlight(ax, 'headlight');
lighting(ax, 'gouraud');

fprintf('Placed %d / %d items.\n', numel(placed), ITEM_COUNT);
if ~isempty(placed)
    fill_height = max([placed.z_top]);
    fprintf('Maximum fill height: %.3f / %.3f\n', fill_height, BIN_SIZE(3));
end

%% ============================= local functions =========================
function box = randomStablePose(box)
% Choose one of the six stable faces, represented by a dimension
% permutation, followed by a continuous random yaw rotation.

    dims = [box.w, box.h, box.d];
    order = randperm(3);
    dims = dims(order);

    box.w = dims(1);
    box.h = dims(2);
    box.d = dims(3);
    box.size = dims;

    yaw = 2 * pi * rand;
    c = cos(yaw);
    s = sin(yaw);
    box.R = [c, -s, 0; s, c, 0; 0, 0, 1];
    box.orientation = [0, 0, sin(0.5 * yaw), cos(0.5 * yaw)];
end

function [box, accepted] = settleCandidate( ...
    box, placed, bin_size, min_support_ratio, tol)
% Randomly choose an XY location, then drop the item vertically onto the
% highest intersecting support footprint.

    accepted = false;
    L = bin_size(1);
    W = bin_size(2);
    H = bin_size(3);

    local_footprint = rectangleFootprint(box.w, box.h, box.R(1:2, 1:2));
    min_rel = min(local_footprint, [], 1);
    max_rel = max(local_footprint, [], 1);

    x_low = -0.5 * L - min_rel(1);
    x_high = 0.5 * L - max_rel(1);
    y_low = -0.5 * W - min_rel(2);
    y_high = 0.5 * W - max_rel(2);

    if x_low > x_high || y_low > y_high
        return;
    end

    center_xy = [ ...
        x_low + (x_high - x_low) * rand, ...
        y_low + (y_high - y_low) * rand];
    footprint = local_footprint + center_xy;

    % The first contact during a vertical drop is the highest top surface
    % whose horizontal footprint overlaps the candidate footprint.
    support_z = 0;
    for i = 1:numel(placed)
        intersection = convexPolygonIntersection( ...
            footprint, placed(i).footprint, tol);
        if polygonArea(intersection) > tol
            support_z = max(support_z, placed(i).z_top);
        end
    end

    z_bottom = support_z;
    z_top = z_bottom + box.d;
    if z_top > H + tol
        return;
    end

    support_ids = zeros(1, 0);

    if support_z > tol
        contact_points = zeros(0, 2);
        total_contact_area = 0;

        for i = 1:numel(placed)
            if abs(placed(i).z_top - support_z) > tol
                continue;
            end

            intersection = convexPolygonIntersection( ...
                footprint, placed(i).footprint, tol);
            area_i = polygonArea(intersection);

            if area_i > tol
                total_contact_area = total_contact_area + area_i;
                contact_points = [contact_points; intersection]; %#ok<AGROW>
                support_ids(end + 1) = i; %#ok<AGROW>
            end
        end

        bottom_area = box.w * box.h;
        if total_contact_area / bottom_area < min_support_ratio
            return;
        end

        contact_points = unique(round(contact_points / tol) * tol, ...
            'rows', 'stable');
        if size(contact_points, 1) < 3
            return;
        end

        centered = contact_points - contact_points(1, :);
        if rank(centered, 1e-10) < 2
            return;
        end

        hull_indices = convhull(contact_points(:, 1), contact_points(:, 2));
        hull = contact_points(hull_indices, :);
        [inside, on_boundary] = inpolygon( ...
            center_xy(1), center_xy(2), hull(:, 1), hull(:, 2));

        if ~(inside || on_boundary)
            return;
        end
    end

    box.position = [center_xy, z_bottom + 0.5 * box.d];
    box.footprint = footprint;
    box.z_bottom = z_bottom;
    box.z_top = z_top;
    box.support_ids = support_ids;
    accepted = true;
end

function polygon = rectangleFootprint(w, h, R2)
% Counter-clockwise rectangle vertices in world XY coordinates, centered at
% the origin before translation.

    local = 0.5 * [
        -w, -h;
         w, -h;
         w,  h;
        -w,  h
    ];
    polygon = local * R2.';
end

function output = convexPolygonIntersection(subject, clipper, tol)
% Sutherland-Hodgman clipping for two counter-clockwise convex polygons.

    output = subject;
    if isempty(output) || isempty(clipper)
        output = zeros(0, 2);
        return;
    end

    for i = 1:size(clipper, 1)
        a = clipper(i, :);
        b = clipper(mod(i, size(clipper, 1)) + 1, :);

        input = output;
        output = zeros(0, 2);
        if isempty(input)
            break;
        end

        s = input(end, :);
        for j = 1:size(input, 1)
            e = input(j, :);
            e_inside = isLeftOrOn(a, b, e, tol);
            s_inside = isLeftOrOn(a, b, s, tol);

            if e_inside
                if ~s_inside
                    output(end + 1, :) = lineIntersection(s, e, a, b); %#ok<AGROW>
                end
                output(end + 1, :) = e; %#ok<AGROW>
            elseif s_inside
                output(end + 1, :) = lineIntersection(s, e, a, b); %#ok<AGROW>
            end
            s = e;
        end
    end

    if ~isempty(output)
        output = removeNearDuplicateVertices(output, tol);
    end
end

function tf = isLeftOrOn(a, b, p, tol)
    edge = b - a;
    rel = p - a;
    tf = edge(1) * rel(2) - edge(2) * rel(1) >= -tol;
end

function p = lineIntersection(p1, p2, q1, q2)
% Intersection of the infinite lines p1-p2 and q1-q2. In the clipping
% algorithm the intersection is known to lie on segment p1-p2.

    r = p2 - p1;
    s = q2 - q1;
    denominator = cross2(r, s);

    if abs(denominator) < 1e-14
        p = 0.5 * (p1 + p2);
        return;
    end

    t = cross2(q1 - p1, s) / denominator;
    p = p1 + t * r;
end

function value = cross2(a, b)
    value = a(1) * b(2) - a(2) * b(1);
end

function polygon = removeNearDuplicateVertices(polygon, tol)
    if size(polygon, 1) <= 1
        return;
    end

    keep = true(size(polygon, 1), 1);
    for i = 2:size(polygon, 1)
        if norm(polygon(i, :) - polygon(i - 1, :)) <= tol
            keep(i) = false;
        end
    end
    polygon = polygon(keep, :);

    if size(polygon, 1) > 1 && ...
            norm(polygon(1, :) - polygon(end, :)) <= tol
        polygon(end, :) = [];
    end
end

function area = polygonArea(polygon)
    if size(polygon, 1) < 3
        area = 0;
    else
        area = polyarea(polygon(:, 1), polygon(:, 2));
    end
end

function drawOpenBin(bin_size)
    L = bin_size(1);
    W = bin_size(2);
    H = bin_size(3);

    x0 = -0.5 * L;
    x1 =  0.5 * L;
    y0 = -0.5 * W;
    y1 =  0.5 * W;

    floor_vertices = [x0 y0 0; x1 y0 0; x1 y1 0; x0 y1 0];
    patch('Vertices', floor_vertices, 'Faces', [1 2 3 4], ...
        'FaceColor', [0.65 0.68 0.72], 'FaceAlpha', 0.28, ...
        'EdgeColor', [0.15 0.15 0.15], 'LineWidth', 1.2);

    walls = {
        [x0 y0 0; x1 y0 0; x1 y0 H; x0 y0 H], ...
        [x1 y0 0; x1 y1 0; x1 y1 H; x1 y0 H], ...
        [x1 y1 0; x0 y1 0; x0 y1 H; x1 y1 H], ...
        [x0 y1 0; x0 y0 0; x0 y0 H; x0 y1 H]
    };

    for i = 1:numel(walls)
        patch('Vertices', walls{i}, 'Faces', [1 2 3 4], ...
            'FaceColor', [0.45 0.55 0.68], 'FaceAlpha', 0.10, ...
            'EdgeColor', [0.18 0.22 0.28], 'LineWidth', 1.0);
    end
end
