clear all; close all; clc;

np = 3; % number of pursuers
ne = 1; % number of evaders
n = np + ne;
vmax = 1; % max velocity for pursuers and evaders

% Calculate bounded Voronoi cells
P = randi(20, [n, 2]) - 10;
sqr_border = [-10, -10, 10, 10; -10, 10, 10, -10];
[vertices, indices, xy] = VoronoiLimit(P(:, 1), P(:, 2), 'bs_ext', sqr_border','figure', 'off');

% Get new indices since VoronoiLimit swapped them around
newInds = getNewInds(P, xy);

inds_p = [];
inds_e = [];
for i = 1:n
    if newInds(i) <= ne
        inds_e = [inds_e, i];
    else
        inds_p = [inds_p, i];
    end
end

% Calculate controls
u_p = pursuer_velocity(vertices, indices, xy, inds_p, inds_e);
u_e = evader_velocity(vertices, indices, xy, inds_e);

% Convert into dx vector
dx = zeros(4*n,1);
for i = 1:np
    dx(4*inds_p(i)-3:4*inds_p(i)-2) = u_p(2*i-1:2*i);
end

for i = 1:ne
    dx(4*inds_e(i)-3:4*inds_e(i)-2) = u_e(2*i-1:2*i);
end

% Plot Voronoi cells
[vx, vy] = voronoi(P(:, 1), P(:, 2));
plot(vx, vy, '-k');
hold on

% Plot pursuer position and velocity (blue is pursuer)
for i = 1:length(inds_p)
    plot([xy(inds_p(i),1), u_p(2*i-1)+xy(inds_p(i),1)], [xy(inds_p(i),2), u_p(2*i)+xy(inds_p(i),2)], '--b');
    plot(xy(inds_p(i),1), xy(inds_p(i),2), 'ob', 'MarkerFaceColor', 'b', 'MarkerSize', 3);
end

% Plot evader position and velocity (red is evader)
for i = 1:length(inds_e)
    plot([xy(inds_e(i),1), u_e(2*i-1)+xy(inds_e(i),1)], [xy(inds_e(i),2), u_e(2*i)+xy(inds_e(i),2)], '--r');
    plot(xy(inds_e(i),1), xy(inds_e(i),2), 'or', 'MarkerFaceColor', 'r', 'MarkerSize', 3);
end

title('Robot Positions, Voronoi Cells, Velocity Directions');
axis equal
xlim([-10, 10]);
ylim([-10, 10]);


function u_e = evader_velocity(vertices, indices, xy, inds_e)
    % inds_e: indices in xy that are evaders
    
    u_e = [];
    
    % Find centroid for each evader cell and go towards it
    for i=inds_e
        cell_shape = polyshape(vertices(indices{i}, 1), vertices(indices{i}, 2));
        [Cvi_x, Cvi_y] = centroid(cell_shape);
        Cvi = [Cvi_x, Cvi_y];
        xe = [xy(i,1), xy(i,2)];
        u_e = [u_e, ((Cvi - xe)/norm(Cvi - xe))];  
        plot(Cvi(1), Cvi(2), 'xr', 'MarkerSize', 10)
    end
    u_e = u_e';
end

function u_p = pursuer_velocity(vertices, indices, xy, inds_p, inds_e)
    % inds_p: indices in xy that are pursuers
    % inds_p: indices in xy that are pursuers
    
    u_p = [];
    for i=inds_p
        p_pos = xy(i,:);
        neighbors_e = [];
        
        % Find neighboring evaders by checking if they have common edges
        for j=inds_e
            s = size(intersect(indices{i}, indices{j}));
            if (s(2) > 1)
                neighbors_e = [neighbors_e, j];
            end
        end

        % If the pursuer has neighboring evaders
        if (length(neighbors_e) >= 1)
            % Find nearest one
            nearest_dist = Inf;
            nearest_edge = [0,0];
            for j = neighbors_e            
                e_pos = xy(j,:);
                if norm(p_pos-e_pos) < nearest_dist
                    nearest_dist = norm(p_pos-e_pos);
                    nearest_edge = intersect(indices{i}, indices{j});
                end
            end

            % Find center of shared edge
            vertex_1 = vertices(nearest_edge(1), :);
            vertex_2 = vertices(nearest_edge(2), :);
            Cbj = .5*(vertex_1+vertex_2);
            plot(Cbj(1), Cbj(2), 'xb', 'MarkerSize', 10)
            hold on
            u_p = [u_p; ((Cbj - p_pos)/norm(Cbj - p_pos))'];

        % If the pursuer has no neighboring evaders
        else
            % Find nearest
            nearest_dist = Inf;
            nearest_pos = [0,0];
            for j = inds_e            
                e_pos = xy(j,:);
                if norm(p_pos-e_pos) < nearest_dist
                    nearest_dist = norm(p_pos-e_pos);
                    nearest_pos = e_pos;
                end
            end

            % Go directly towards it
            u_p = [u_p; ((nearest_pos - p_pos)/norm(nearest_pos - p_pos))'];
        end
    end
end

function newInds = getNewInds(P, xy)
    % Swap indices
    newInds = zeros(size(P, 1), 1);
    for i=1:size(xy, 1)
        for j=1:size(P, 1)
            if (xy(i, :) == P(j, :))
                newInds(i) = j;
            end
        end
    end
end