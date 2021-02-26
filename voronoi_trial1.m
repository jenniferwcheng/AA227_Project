% 
n_p = 1; % number of pursuers
n_e = 1; % number of evaders
vmax = 1; % max velocity for pursuers and evaders
% Moving through R2

P = [0.5 0; 0 0.5; -0.5 -0.5; -0.2 -0.1; -0.1 0.1; 0.1 -0.1; 0.1 0.1];
calculate_evader_velocity(P(:, 1), P(:, 2), 4:5)

function u_e = calculate_evader_velocity(pos_x, pos_y, inds_e)
% pos_x: x-coordinates of all positions in a column vector
% pos_y: y-coordinates of all positions in a column vector
% inds_e: range of indices in pos_x and pos_y that are evaders (eg. 1:3)

[vx, vy] = voronoi(pos_x, pos_y);
[vertices, indices] = voronoin([pos_x pos_y]);
u_e = [];

for i=inds_e
    cell_shape = polyshape(vertices(indices{i}, 1), vertices(indices{i}, 2));
    Cvi = centroid(cell_shape);
    xe = [pos_x(i), pos_y(i)];
    u_e = [u_e; (Cvi - xe)/norm(Cvi - xe)];    
end

end

function u_p = calculate_pursuer_velocity(pos_x, pos_y, inds_p, inds_e)
% pos_x: x-coordinates of all positions in a column vector
% pos_y: y-coordinates of all positions in a column vector
% inds_p: range of indices in pos_x and pos_y that are pursuers (eg. 1:3)

[vertices, indices] = voronoin([pos_x pos_y]);

for i=inds_p
    neighbors_e = [];
    for j=inds_e
        s = size(intersect(indices{i}, indices{j}));
        if (s(2) > 1)
            neighbors_e = [neighbors_e, j];
        end
    end
    
    % if the pursuer has neighboring evaders
    if (length(neighbors_e) > 1)
    % if the pursuer has no neighboring evaders
    else
    end
end

end