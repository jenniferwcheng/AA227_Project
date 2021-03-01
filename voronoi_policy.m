n_p = 3; % number of pursuers
n_e = 1; % number of evaders
n = n_p + n_e;
vmax = 1; % max velocity for pursuers and evaders

% Calculate evader and pursuer velocities
P = randi(20, [4, 2]) - 10
u_p = pursuer_velocity(P(:, 1), P(:, 2), 1:n_p, n_p+1:n)
u_e = evader_velocity(P(:, 1), P(:, 2), n_p+1:n)

% Plot robot initial positions and Voronoi cells
figure(1);
plot(P(1:n_p, 1), P(1:n_p, 2), 'or', 'MarkerFaceColor', 'r', 'MarkerSize', 3);
hold on;
plot(P(n_p+1:end, 1), P(n_p+1:end, 2), 'ob', 'MarkerFaceColor', 'b', 'MarkerSize', 3);
[vx, vy] = voronoi(P(:, 1), P(:, 2));
plot(vx, vy, '-k');
title('Robot Init Positions, Voronoi Cells, Velocity Directions');
axis equal
xlim([-10, 10]);
ylim([-10, 10]);

% Plot pursuer velocity (red is pursuer)
for i=1:n_p
    plot([P(i, 1), u_p(2*i-1)], [P(i, 2), u_p(2*i)], '--r'); 
end

% Plot pursuer velocity (red is pursuer)
for i=n_p+1:n
    plot([P(i, 1), u_e(2*(i-n_p)-1)], [P(i, 2), u_e(2*(i-n_p))], '--b');
end

function u_e = evader_velocity(pos_x, pos_y, inds_e)
% pos_x: x-coordinates of robot positions (column)
% pos_y: y-coordinates of robot positions (column)
% inds_e: indices in pos_x and pos_y that are evaders

sqr_border = [-10, -10, 10, 10; -10, 10, 10, -10];
[vertices, indices, ~] = VoronoiLimit(pos_x, pos_y, 'bs_ext', sqr_border');

% PROBLEM WITH INDEXING - Indexes of robot positions do not match indices
% of Voronoi cell vertices stored in the indices vector
% VoronoiLimit outputs polygon vertices in CCW order

u_e = [];
for i=inds_e
    cell_shape = polyshape(vertices(indices{i}, 1), vertices(indices{i}, 2));
    [Cvi_x, Cvi_y] = centroid(cell_shape);
    Cvi = [Cvi_x, Cvi_y];
    xe = [pos_x(i), pos_y(i)];
    u_e = [u_e, ((Cvi - xe)/norm(Cvi - xe))];    
end
u_e = u_e';
end

function u_p = pursuer_velocity(pos_x, pos_y, inds_p, inds_e)
% pos_x: x-coordinates of robot positions (column)
% pos_y: y-coordinates of robot positions (column)
% inds_p: indices in pos_x and pos_y that are pursuers

sqr_border = [-10, -10, 10, 10; -10, 10, 10, -10];
[vertices, indices, ~] = VoronoiLimit(pos_x, pos_y, 'bs_ext', sqr_border');

u_p = [];
for i=inds_p
    p_pos = [pos_x(i), pos_y(i)];
    neighbors_e = [];
    for j=inds_e
        s = size(intersect(indices{i}, indices{j}));
        if (s(2) > 1)
            neighbors_e = [neighbors_e, j];
        end
    end
    
    % if the pursuer has neighboring evaders
    if (length(neighbors_e) > 1)
        % find nearest one
        nearest_dist = Inf;
        nearest_edge = [0,0];
        for j = neighbors_e            
            e_pos = [pos_x(j), pos_y(j)];
            if norm(p_pos-e_pos) < nearest_dist
                nearest_dist = norm(p_pos-e_pos);
                nearest_edge = intersect(indices{i}, indices{j});
            end
        end

        vertex_1 = vertices(nearest_edge(1));
        vertex_2 = vertices(nearest_edge(2));
        Cbj = .5*(vertex_1+vertex_2);
        u_p = [u_p; ((Cbj - p_pos)/norm(Cbj - p_pos))'];
        
    % if the pursuer has no neighboring evaders
    else
        % find nearest
        nearest_dist = Inf;
        nearest_pos = [0,0];
        for j = neighbors_e            
            e_pos = [pos_x(j), pos_y(j)];
            if norm(p_pos-e_pos) < nearest_dist
                nearest_dist = norm(p_pos-e_pos);
                nearest_pos = e_pos;
            end
        end
        
        u_p = [u_p; ((nearest_pos - p_pos)/norm(nearest_pos - p_pos))'];
    end
end
end