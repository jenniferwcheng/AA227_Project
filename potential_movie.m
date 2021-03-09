%% Setup
clear all; close all; clc;

%----Parameters-----------
global ne;
global np;
ne = 1; % Number of evaders
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
method = 0; % 0 for potential, 1 for Voronoi
save_video = 0; % 1 to plot in real time and save video
monte_carlo = 0; % 1 - on, 0 - off

t_end = 60; % [s] length of simulation time

%----Initial conditions------

% Random positions:
x0 = grid_size/2*rand([n*dim,1]) - grid_size/4;
x0(3:dim:end) = 0; % zero velocity
x0(4:dim:end) = 0; % zero acceleration

% x0 = [0; 0; 0; 0; -4; -4; 0; 0; 4; 4; 0; 0; 5; -4; 0; 0; -4; 4; 0; 0]; % square

%% Run once with equi-spaced times
% load('initial.mat')
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
    dx = potential_fun(t,x, vmax, amax, ne, np, grid_size);

    % Plot evader (assume only 1)
            plot(x(1), x(2), '.r', 'MarkerSize', 20) % Evader position
            hold on
%             plot([x(1) x(1)+dx(1)], [x(2) x(2)+dx(2)], '--r') % Evader velocity
            
            % Extract pursuer x and y locations (assume 1 evader)
            plotx = x(5:4:end-1);
            ploty = x(6:4:end);
            
            % Plot pursuers
            for i = 1:length(plotx)
                plot(plotx(i), ploty(i), '.b', 'MarkerSize', 20) % Pursuer locations
%                 plot([plotx(i) plotx(i) + dx(4*i-3)], [ploty(i) ploty(i)+dx(4*i-2)], '--b') % Pursuer velocity
            end
            
            xlim([-grid_size/2 grid_size/2])
            ylim([-grid_size/2 grid_size/2])
            xlabel('x1 [m]')
            ylabel('x2 [m]')
            title('Current Position')
            grid on
            drawnow
            hold off
    
    global F;
    F = [F; getframe(gcf)];
end

%% make video
writerObj = VideoWriter('multiple_pursuers_test.avi');
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
    k = 1e-3; % tune this - need this to be small so that pursuers can surround evader
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


