function dx = ode_fun(t, x, method, save_video, vmax, amax, ne, np, grid_size)
% INPUTS:
    % t - current time
    % x - states
    % method - 0 for potential, 1 for Voronoi
    % save_video - boolean save/not save video
    % vmax - maximum velocity [m/s]
    % amax - maximum acceleration [m/s^2]
    % ne - number of evaders
    % np - number of pursuers
    % grid_size - size of environment [m]
% OUTPUTS:
    % dx - state derivatives

    % Check which method to use
    if method == 0
        dx = potential_fun(t,x, vmax, amax, ne, np, grid_size);
    else
        clf;
        [dx, inds_p, inds_e, xy, u_p, u_e] = voronoi_fun(t, x, ne, np, grid_size);
        t
    end
    
    if save_video
        if method == 0 % Potential
            % plot in real time
            plot(x(1), x(2), '.r', 'MarkerSize', 20) % plot evader
            hold on
            plotx = x(5:4:end-1);
            ploty = x(6:4:end);
            plot(plotx,ploty, '.b', 'MarkerSize', 20) % plot pursuers
            xlim([-grid_size/2 grid_size/2])
            ylim([-grid_size/2 grid_size/2])
            xlabel('x1')
            ylabel('x2')
            title('Current Position')
            grid on
            drawnow
            hold off
        else
            % Plot Voronoi cells
            px = x(1:4:end);
            py = x(2:4:end);
            [vx, vy] = voronoi(px,py);
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
            xlim(grid_size/2*[-1, 1]);
            ylim(grid_size/2*[-1, 1]);
            drawnow
            hold off
        end

        global F;
        F = [F; getframe(gcf)];
    end
%     x;
%     dx
end

function [dx, inds_p, inds_e, xy, u_p, u_e] = voronoi_fun(t, x, ne, np, grid_size)
    n = ne+np;
    
    % Bound environment
%     sqr_border = grid_size/2*[-1, -1, 1, 1; -1, 1, 1, -1];
    px = x(1:4:end);
    py = x(2:4:end);
    
    lower = [-grid_size/2, -grid_size/2];
    upper = [grid_size/2, grid_size/2];
    [vertices, indices] = bounded_voronoi(lower, upper, [px, py]);
    xy = [px, py];
%     [vertices, indices, xy] = VoronoiLimit(px, py, 'bs_ext', sqr_border','figure', 'off');

%     % Get new indices since VoronoiLimit swapped them around
%     newInds = getNewInds([px py], xy);
% 
%     inds_p = [];
%     inds_e = [];
%     for i = 1:n
%         if newInds(i) <= ne
%             inds_e = [inds_e, i];
%         else
%             inds_p = [inds_p, i];
%         end
%     end
    
    inds_e = 1:ne;
    inds_p = ne+1:n;

    % Calculate controls
    u_p = pursuer_velocity(vertices, indices, xy, inds_p, inds_e);
    u_e = evader_velocity(vertices, indices, xy, inds_e);
    
%     u_p = u_p;
%     u_e = u_e;

    % Convert into dx vector
    dx = zeros(4*n,1);
    for i = 1:np
        dx(4*inds_p(i)-3:4*inds_p(i)-2) = u_p(2*i-1:2*i);
    end

    for i = 1:ne
        dx(4*inds_e(i)-3:4*inds_e(i)-2) = u_e(2*i-1:2*i);
    end
end

function [dx] = potential_fun(t,x, vmax, amax, ne, np, grid_size)   
% INPUTS:
    % t - current time
    % x - states
    % vmax - maximum velocity [m/s]
    % amax - maximum acceleration [m/s^2]
    % ne - number of evaders
    % np - number of pursuers
    % grid_size - size of environment [m]
% OUTPUTS:
    % dx - state derivatives
    
    % Shape the state and derivative into
    % dims x num_robots matrices
    dims = 4;
    n = ne + np;
    num_evaders = ne; 
    num_pursuers = n - num_evaders;
   
    X = reshape(x, dims, n);
    forces = zeros(dims/2, n);
    wall_forces = zeros(dims/2, n);
    
    % Calculate the force for the ith robot
    for i = 1:n
        x1 = X(1:2, i);
        
        forces(:, i) = forces(:,i) + wall_force(x1, grid_size);
        
        % Account for force from any other robot
        for j = 1:n
            if i ~= j 
                x2 = X(1:2, j);

                % Consider each possible evader, pursuer pairing
                if i <= num_evaders 
                    % Evader, evader
                    if j <= num_evaders
                        forces(:, i) = forces(:, i) + evader_evader_force(x1, x2);
                    % Evader, pursuer
                    else 
                        forces(:, i) = forces(:, i) + evader_pursuer_force(x1, x2);
                    end            
                else
                    % Pursuer, evader
                    if j <= num_evaders
                        forces(:, i) = forces(:, i) + pursuer_evader_force(x1, x2);
                    % Pursuer, pursuer
                    else
                        forces(:, i) = forces(:, i) + pursuer_pursuer_force(x1, x2);
                    end
                end
            end
        end
    end
    
    % Return the vectorized version of the derivative
    dX = zeros(dims, n);
    dX(1:2, :) = X(3:4, :); % Change position by the velocity
    dX(3:4, :) = forces - 0.05*X(3:4, :); % Change velocity by the force - friction, assume m = 1 for all robots
    
    % Limit acceleration and velocities
    for i = 1:n
        % If velocities are >= max allowable, don't accelerate
        % TODO: fix this to account for direction
        if(X(3,i) >= vmax && dX(3,i) > 0)
            dX(3,i) = 0;
        elseif (X(3,i) <= -vmax && dX(3,i) < 0)
            dX(3,i) = 0;
        end
            

        if(X(4,i) <= -vmax && dX(4,i) < 0)
            dX(4,i) = 0;
        elseif(X(4,i) >= vmax && dX(4,i) > 0)
            dX(4,i) = 0;
        end
         

        % Method 1: If accelerations are >= max allowable, saturate
%         if(abs(dX(3,i)) >= amax)
%             dX(3,i) = amax*sign(dX(3,i));
%         end
%         
%         if(abs(X(4,i)) >= vmax)
%             dX(4,i) = amax*sign(dX(4,i));
%         end

    % Method 2: Normalize accelerations
        anorm = norm(dX(3:4,i));
        if anorm > amax
            dX(3:4,i) = dX(3:4,i)./(anorm/amax);
        end
    end
    
    % Account for boundary
%     dX(3:4, :) = dX(3:4,:) + wall_forces;
    
    % Reshape into a vector 
    dx = dX(:);
end

% Force on pursuer at x1 given x1 is a pursuer, x2 is another pursuer
function [force] = pursuer_pursuer_force(x1, x2)
    r = x2 - x1;
    k = 0.01; % tune this
%     force = -k/norm(r)*r;
    force = -k/norm(r)^3*r;
end

% Force on pursuer at x1 given x1 is a pursuer, x2 is an evader
function [force] = pursuer_evader_force(x1, x2)
    r = x2 - x1;
   
    % tune these
    k = 0.8;
    
%     force = k/norm(r)*r;
    force = k/norm(r)^3*r;

    % Method 2: piecewise?
%     kn = 2; % near evader
%     kf = 3; % far from evader
    
%     if norm(r) > 4 % If far from evader, use proportional gain
%         force = kf*r;
%     else % If close to evader, use potential field
%         force = kn/norm(r)^3*r;
%     end
end

% Force on evader at x1 given x1 is an evader, x2 is a pursuer
function [force] = evader_pursuer_force(x1, x2)
    r = x2 - x1;
    k = 0.5; % tune this
    
%     force = -k/norm(r)*r;
    force = -k/norm(r)^3*r;
end

% Force on evader at x1 given x1 is an evader, x2 is an evader
function [force] = evader_evader_force(x1, x2)
    r = x2 - x1;
    k = 0; % tune this
    
%     force = -k/norm(r)*r;
    force = -k/norm(r)^3*r;
end

function [force] = wall_force(x1, grid_size)
    % Environment is [-m m] x [-m m]
    m = grid_size/2;
    
    x = x1(1);
    y = x1(2);
    
    k = 1.5; % Tune this
    
    force = 0;
    
    thres = m*0.85; % 1.5 meters away from wall
    
    % Check if close to wall
    if y > thres % Top
        r = [0; m - y];
        force = force - k/norm(r)^3*r;
    end
    if y < -thres % Bottom
        r = [0; -m - y];
        force = force - k/norm(r)^3*r;
    end
    if x < -thres % Left
        r = [-m - x; 0];
        force = force - k/norm(r)^3*r;
    end
    if x > thres % Right
        r = [m - x; 0];
        force = force - k/norm(r)^3*r;
    end
end

function u_e = evader_velocity(vertices, indices, xy, inds_e)
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

function u_p = pursuer_velocity(vertices, indices, xy, inds_p, inds_e)
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
            hold on;
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
%     augmented_points = unique(augmented_points, 'row');
    [v, c] = voronoin(augmented_points);
    c = c(1:size(points, 1));
    
    % Plot to demonstrate that this works
    %     voronoi(augmented_points(:, 1), augmented_points(:, 2));
    %     hold on
    %     plot(points(:, 1), points(:, 2), '*r');
    %     plot([lower(1), lower(1), upper(1), upper(1)], [lower(2), upper(2), upper(2), lower(2)])
end
