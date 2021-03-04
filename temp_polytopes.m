points = rand(10, 2)*2 - 1; %[0.5 0.5; 0.5 -0.5; -0.5 0.5; -0.5 -0.5];

% Define your region
lower = [-1, -1];
upper = [1, 1];

% Test finding a voronoi decomposition in a bounded region
[v, c] = bounded_voronoi(lower, upper, points);

% Lower is the bottom left corner of the rectangle
% Upper is the top right corner of the rectangle
% These points will enforce that the tiles are within the desired rectangle
function points = augment_point(lower, upper, point)
    left_point = point - 2*[point(1) - lower(1), 0];
    right_point = point + 2*[upper(1) - point(1), 0];
    bottom_point = point - 2*[0, point(2) - lower(2)];
    top_point = point + 2*[0, upper(2) - point(2)];
    points = [left_point; right_point; bottom_point; top_point];
end

function [v, c] = bounded_voronoi(lower, upper, points)
    % Initialize the list of points
    augmented_points = points;
    
    % For each point, add the four boundary points associated with it
    for i = 1:size(points, 1)
        point = points(i, :);
        augment_point(lower, upper, point);
        augmented_points = [augmented_points; augment_point(lower, upper, point)];
    end
    
    % Decompose the space using the extra points then truncate to the first
    % num_points nodes
    [v, c] = voronoin(augmented_points);
    c = c{1:size(points, 1)};
    
    % Plot to demonstrate that this works
    %     voronoi(augmented_points(:, 1), augmented_points(:, 2));
    %     hold on
    %     plot(points(:, 1), points(:, 2), '*r');
    %     plot([lower(1), lower(1), upper(1), upper(1)], [lower(2), upper(2), upper(2), lower(2)])
end
