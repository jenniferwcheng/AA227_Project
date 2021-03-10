%% Setup
clear all; close all; clc;

%----Parameters-----------
global ne;
global np;
ne = 2; % Number of evaders
np = 4; % Number of pursuers
n = ne + np; % Total number of robots
dim = 4; % Order of states

capture_radius = 0.2; % [m] (If changing this value, remember to change in termEvent.m as well)
global caught; % Global variable to determine whether an evader is caught: 0 - not caught, 1 - caught
caught = zeros(ne, 1);
vmax = 1; % [m/s] Same for both methods
amax = 10; % [m/s^2] 
grid_size = 20; % [m] size of environment (length) -> area = grid_size^2
global F; % For video

% Flags
method = 1; % 0 for potential, 1 for Voronoi
save_video = 0; % 1 to plot in real time and save video
monte_carlo = 0; % 1 - on, 0 - off

t_end = 51; % [s] length of simulation time

%----Initial conditions------

% Random positions:
% x0 = grid_size/2*rand([n*dim,1]) - grid_size/4;
% x0(3:dim:end) = 0; % zero velocity
% x0(4:dim:end) = 0; % zero acceleration

% x0 = [0; 0; 0; 0; -4; -4; 0; 0; 4; 4; 0; 0; 5; -4; 0; 0; -4; 4; 0; 0]; % square

%% Run once with equi-spaced times
load('initial.mat')
tspan = 0:.1:t_end;
Opt = odeset('Events', @termEvent); % Terminate when within capture radius
[t_all, x_all] = ode23(@(t,x) ode_fun(t,x, method, save_video, vmax, amax, ne, np, grid_size),tspan, x0, Opt);

%% Make movie
figure;

% reset globals
global caught;
caught = zeros(ne, 1);
global F;
F = [];

for i=1:length(t_all)
    t = t_all(i);
    x = x_all(i,:)';
    [dx, inds_p, inds_e, xy, u_p, u_e, aug_pts] = voronoi_fun(t, x, ne, np, grid_size, save_video);

    px = x(1:4:end);
    py = x(2:4:end);
    n = ne + np;

    % Remove caught evaders (don't want to plot their cells)
    for i = 1:ne
        if caught(i)
            aug_pts(i,:) = [];
        end
    end

    % Plot bounded Voronoi cells
    voronoi(aug_pts(:, 1), aug_pts(:, 2), 'k');
    hold on

    % Plot pursuer position and velocity (blue is pursuer)
    for i = ne+1:n
        plot([x(4*i-3), x(4*i-3) + dx(4*i-3)], [x(4*i-2), x(4*i-2) + dx(4*i-2)], '--b');
        plot(x(4*i-3), x(4*i-2), 'ob', 'MarkerFaceColor', 'b', 'MarkerSize', 3);

        % For single evader
%                 plot([xy(inds_p(i),1), u_p(2*i-1)+xy(inds_p(i),1)], [xy(inds_p(i),2), u_p(2*i)+xy(inds_p(i),2)], '--b');
%                 plot(xy(inds_p(i),1), xy(inds_p(i),2), 'ob', 'MarkerFaceColor', 'b', 'MarkerSize', 3);
    end

    % Plot evader position and velocity (red is evader)
    for i = 1:ne
        if ~caught(i)
        plot([x(4*i-3), x(4*i-3) + dx(4*i-3)], [x(4*i-2), x(4*i-2) + dx(4*i-2)], '--r');
        end
        plot(x(4*i-3), x(4*i-2), 'or', 'MarkerFaceColor', 'r', 'MarkerSize', 3);
        % For single evader
%                 plot([xy(inds_e(i),1), u_e(2*i-1)+xy(inds_e(i),1)], [xy(inds_e(i),2), u_e(2*i)+xy(inds_e(i),2)], '--r');
%                 plot(xy(inds_e(i),1), xy(inds_e(i),2), 'or', 'MarkerFaceColor', 'r', 'MarkerSize', 3);
    end

    title('Robot Positions, Voronoi Cells, Velocity Directions');
    axis equal
    xlim(grid_size/2*[-1, 1])
    ylim(grid_size/2*[-1, 1])
    xlabel('x1 [m]')
    ylabel('x2 [m]')
    grid on
    drawnow
    hold off
    
    global F;
    F = [F; getframe(gcf)];
end

%% make video
writerObj = VideoWriter('voronoi_video.avi');
writerObj.FrameRate = 10;
open(writerObj);
% write the frames to the video
for i=1:length(F)
    % convert the image to a frame
    frame = F(i) ;    
    writeVideo(writerObj, frame);
end
close(writerObj);

%% functions
function [dx, inds_p, inds_e, xy, u_p, u_e, aug_pts] = ...
    voronoi_fun(t, x, ne, np, grid_size, save_video)
    global caught;
    n = ne+np;
    
    % Bound environment
    px = x(1:4:end); % Extract x and y positions
    py = x(2:4:end);
    xy = [px, py];
    
    % Evader and pursuer indices    
    inds_e = 1:ne;
    inds_p = ne+1:n;
    
    % Remove caught evaders
    for i = ne:-1:1
        if caught(i)
            xy(i,:) = [];
            inds_e(:,i:end) = inds_e(i:end) - 1;
            inds_e(:,i) = [];
        end
    end
    
    % Shift pursuer indices by number of evaders caught
    inds_p = inds_p - sum(caught);
    
    lower = [-grid_size/2, -grid_size/2]; % Lower left corner
    upper = [grid_size/2, grid_size/2]; % Upper right corner
    
    [vertices, indices, aug_pts] = bounded_voronoi(lower, upper, xy);

    % Calculate controls
    u_p = pursuer_velocity(vertices, indices, xy, inds_p, inds_e, save_video);
    u_e = evader_velocity(vertices, indices, xy, inds_e, save_video);
    
    % Adjust pursuer/evader speeds
%     u_p = u_p;
%     u_e = u_e;

    % Convert into dx vector
    dx = zeros(4*n,1);
    
    % Shift pursuer indices back in their original positions
    inds_p = inds_p + sum(caught);
    
    % Put pursuer controls in correct position in dx vector
    for i = 1:np
        dx(4*inds_p(i)-3:4*inds_p(i)-2) = u_p(2*i-1:2*i);
    end

    
    % Put evader indices back in their origional positions (requires some
    % wack indexing)
    j = 1;
    for i = 1:ne-sum(caught)
        while caught(j)
            j = j + 1;
        end
        dx(4*j-3:4*j-2) = u_e(2*i-1:2*i);
        j = j + 1;
    end
    
    checkIfCaught(t,x);
end

function u_e = evader_velocity(vertices, indices, xy, inds_e, save_video)
    % inds_e: indices in xy that are evaders
    u_e = [];
    eps = 1e-2;
    
    % Find centroid for each evader cell and go towards it
    for i=inds_e
        cell_shape = polyshape(vertices(indices{i}, 1), vertices(indices{i}, 2));
        [Cvi_x, Cvi_y] = centroid(cell_shape);
        Cvi = [Cvi_x, Cvi_y];
        xe = [xy(i,1), xy(i,2)];
        u_e = [u_e, ((Cvi - xe)/(norm(Cvi - xe) + eps))];  
        plot(Cvi(1), Cvi(2), 'xr', 'MarkerSize', 10)
        hold on;
    end
    u_e = u_e';
end

function u_p = pursuer_velocity(vertices, indices, xy, inds_p, inds_e, save_video)
    % inds_p: indices in xy that are pursuers
    % inds_p: indices in xy that are pursuers
    u_p = [];
    eps = 1e-2;
    
    for i=inds_p
        p_pos = xy(i,:);
        neighbors_e = [];
        
        % Find neighboring evaders by checking if they have common edges
        for j=inds_e
            s = intersect(indices{i}, indices{j});
            if (length(s) > 1)
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
            hold on;
            u_p = [u_p; ((Cbj - p_pos)/(norm(Cbj - p_pos) + eps))'];

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
            u_p = [u_p; ((nearest_pos - p_pos)/(norm(nearest_pos - p_pos) + eps))'];
        end
    end
end

function points = augment_point(lower, upper, point)
    % Add four points equidistant from agent to each border
    left_point = point - 2*[point(1) - lower(1), 0];
    right_point = point + 2*[upper(1) - point(1), 0];
    bottom_point = point - 2*[0, point(2) - lower(2)];
    top_point = point + 2*[0, upper(2) - point(2)];
    points = [left_point; right_point; bottom_point; top_point];
end

function [v, c, augmented_points] = bounded_voronoi(lower, upper, points)
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
    c = c(1:size(points, 1));
    
    % Plot to demonstrate that this works
    %     voronoi(augmented_points(:, 1), augmented_points(:, 2));
    %     hold on
    %     plot(points(:, 1), points(:, 2), '*r');
    %     plot([lower(1), lower(1), upper(1), upper(1)], [lower(2), upper(2), upper(2), lower(2)])
end

function [] = checkIfCaught(t,x)
    % Update caught vector
    global caught;
    global np;
    global ne;
    capture_radius = 0.2; % [m]
    
    % Extract evader and pursuer locations
    ex = x(1:4:4*ne);
    ey = x(2:4:4*ne+1);
    px = x(4*ne+1:4:end);
    py = x(4*ne+2:4:end);
    
    % Check for each evader if there is a pursuer close to it
    for i = 1:ne
        if ~caught(i) % Only check for evaders that haven't been caught
            for j = 1:np
                r = norm([ex(i); ey(i)] - [px(j); py(j)]);
                if r <= capture_radius % Check if there is a pursuer within capture radius
                    caught(i) = 1;
                    break
                end
            end
        end
    end
end
