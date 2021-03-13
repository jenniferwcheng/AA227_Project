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
    global caught;
    
    % Check which method to use
    if method == 0
        dx = potential_fun(t,x, vmax, amax, ne, np, grid_size);
    else
        [dx, inds_p, inds_e, xy, u_p, u_e, aug_pts] = voronoi_fun(t, x, ne, np, grid_size, save_video);
    end
    
    % Plot in real time
    if save_video
        % Potential
        if method == 0 
            % Plot evader 
            for i = 1:ne+np
                if i <= ne
                    plot(x(4*i-3), x(4*i-2), '.r', 'MarkerSize', 20) % Evader position
                    hold on
                else
                    plot(x(4*i-3), x(4*i-2), '.b', 'MarkerSize', 20) % Pursuer locations
                end
            end
            
            xlim([-grid_size/2 grid_size/2])
            ylim([-grid_size/2 grid_size/2])
            xlabel('x1 [m]')
            ylabel('x2 [m]')
            title('Current Position')
            grid on
            drawnow
            hold off
        % Voronoi
        else
            % Plot Voronoi cells
            px = x(1:4:end);
            py = x(2:4:end);
            n = ne + np;

            % Plot bounded Voronoi cells
            voronoi(aug_pts(:, 1), aug_pts(:, 2), 'k');
            hold on

            % Plot pursuer position and velocity (blue is pursuer)
            for i = ne+1:n
                plot([x(4*i-3), x(4*i-3) + dx(4*i-3)], [x(4*i-2), x(4*i-2) + dx(4*i-2)], '--b');
                plot(x(4*i-3), x(4*i-2), 'ob', 'MarkerFaceColor', 'b', 'MarkerSize', 3);
            end

            % Plot evader position and velocity (red is evader)
            for i = 1:ne
                if ~caught(i)
                    plot([x(4*i-3), x(4*i-3) + dx(4*i-3)], [x(4*i-2), x(4*i-2) + dx(4*i-2)], '--r');
                end
                plot(x(4*i-3), x(4*i-2), 'or', 'MarkerFaceColor', 'r', 'MarkerSize', 3);
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
        end

%         global F;
%         F = [F; getframe(gcf)];
    end
%     t
%     x;
%     dx
end

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
        if save_video
            plot(Cvi(1), Cvi(2), 'xr', 'MarkerSize', 10)
            hold on;
        end
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
            if save_video
                plot(Cbj(1), Cbj(2), 'xb', 'MarkerSize', 10)
                hold on;
            end
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
    global caught;
    dims = 4;
    n = ne + np;
    num_evaders = ne; 
    num_pursuers = np;
   
    X = reshape(x, dims, n);
    forces = zeros(dims/2, n);
    wall_forces = zeros(dims/2, n);
    
    % Find closest evader
    % Extract evader and pursuer locations
    ex = x(1:4:4*ne);
    ey = x(2:4:4*ne+1);
    px = x(4*ne+1:4:end);
    py = x(4*ne+2:4:end);
    
    % Find the nearest evader
    closest = zeros(np, 1);
    for i = 1:np
        currClosest = Inf;
        for j = 1:ne
            if ~caught(j)
                dist = norm([ex(j) ey(j)] - [px(i) py(i)]);
                if dist < currClosest
                    currClosest = dist;
                    closest(i) = j;
                end
            end
        end
    end
    
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
                        if ~caught(j)
                            forces(:, i) = forces(:, i) + evader_evader_force(x1, x2);
                        end
                    % Evader, pursuer
                    else 
                        forces(:, i) = forces(:, i) + evader_pursuer_force(x1, x2);
                    end            
                else
                    % Pursuer, evader
                    if j <= num_evaders 
                        if ~caught(j) && j == closest(i-ne)
                            forces(:, i) = forces(:, i) + pursuer_evader_force(x1, x2);
                        end
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
    dX(3:4, :) = forces; % Change velocity by the force - friction, assume m = 1 for all robots
    
    % Limit acceleration and velocities
    for i = 1:n
        % If velocities are >= max allowable, don't accelerate
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
    
    % Reshape into a vector 
    dx = dX(:);
    
    % If caught, don't move
    for i = 1:ne
        if caught(i)
            dx(4*i-3:4*i) = zeros(4,1);
            x(4*i-1:4*i) = zeros(2,1);
        end
    end
end

% Force on pursuer at x1 given x1 is a pursuer, x2 is another pursuer
function [force] = pursuer_pursuer_force(x1, x2)
    r = x2 - x1;
    k = 0; % tune this - need this to be small so that pursuers can surround evader
    % If too small, pursuers will collide with each other
    eps = 1e-2;
    
%     force = -k/norm(r)*r;
    force = -k/(norm(r) + eps)^3*r;
end

% Force on pursuer at x1 given x1 is a pursuer, x2 is an evader
function [force] = pursuer_evader_force(x1, x2)
    r = x2 - x1;
   
    % tune these
    k = 0.8; % the higher k is, the easier it is to catch evader
    % doesn't really increase due to multiple pursuers
    % Lower effect on motion than function below
    eps = 1e-2;
    c = 0.1; % Best values are 0.1-0.5
    
%     force = k/norm(r)*r;
    force = k/(norm(r) + eps)^3*r + c*r/(norm(r) + eps);

    % Method 2: piecewise?
    
    if norm(r) > 1 % If far from evader, force is proportional to distance
%         force = r - 3.8506;
        force = r - 0.1;
%         force = r;
%     else % If close to evader, use potential field
%         force = k/(norm(r) + eps)^3*r;
    end
end

% Force on evader at x1 given x1 is an evader, x2 is a pursuer
function [force] = evader_pursuer_force(x1, x2)
    r = x2 - x1;
    k = 0.5; % tune this - the higher this is, the harder it is to catch pursuers?
    eps = 1e-2;
    
%     force = -k/norm(r)*r;
    force = -k/(norm(r) + eps)^3*r;
end

% Force on evader at x1 given x1 is an evader, x2 is an evader
function [force] = evader_evader_force(x1, x2)
    r = x2 - x1;
    k = 0; % tune this - not having multiple evaders
    eps = 1e-2;
    
%     force = -k/norm(r)*r;
    force = -k/(norm(r) + eps)^3*r;
end

function [force] = wall_force(x1, grid_size)
    % Environment is [-m m] x [-m m]
    m = grid_size/2;
    
    x = x1(1);
    y = x1(2);
    
    k = 3; % Tune this - what are the effects of this?
    % Causes very sharp turns when high
    eps = 1e-2;
    
    force = 0;
    
    thres = m*0.7; % 1.5 meters away from wall
    
    % Check if close to wall
    if y > thres % Top
        r = [0; m - y];
        force = force - k/(norm(r) + eps)^3*r;
    end
    if y < -thres % Bottom
        r = [0; -m - y];
        force = force - k/(norm(r) + eps)^3*r;
    end
    if x < -thres % Left
        r = [-m - x; 0];
        force = force - k/(norm(r) + eps)^3*r;
    end
    if x > thres % Right
        r = [m - x; 0];
        force = force - k/(norm(r) + eps)^3*r;
    end
end

