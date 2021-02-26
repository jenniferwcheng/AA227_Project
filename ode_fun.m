function dx = ode_fun(t, x, method, vmax, amax, ne, np)
% INPUTS:
    % t - current time
    % x - states
    % method - 0 for potential, 1 for Voronoi
    % vmax - maximum velocity [m/s]
    % amax - maximum acceleration [m/s^2]
    % ne - number of evaders
    % np - number of pursuers
% OUTPUTS:
    % dx - state derivatives

    % check which method to use
    if method == 0
        dx = potential(t,x, vmax, amax, ne, np);
    else
        dx = voronoi(t,x);
    end
    
    % plot in real time
    plot(x(1), x(2), '.r', 'MarkerSize', 20) % plot evader
    hold on
    plotx = x(5:4:end-1);
    ploty = x(6:4:end);
    plot(plotx,ploty, '.b', 'MarkerSize', 20) % plot pursuers
    xlim([-20 20])
    ylim([-20 20])
    drawnow
    hold off
end

function [dx] = voronoi(t,x)
    % TODO: voronoi controls
end

function [dx] = potential(t,x, vmax, amax, ne, np)   
% INPUTS:
    % t - current time
    % x - states
    % vmax - maximum velocity [m/s]
    % amax - maximum acceleration [m/s^2]
    % ne - number of evaders
    % np - number of pursuers
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
    
    % Calculate the force for the ith robot
    for i = 1:n
        x1 = X(1:2, i);
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
    dX(3:4, :) = forces; % Change velocity by the force, assume m = 1 for all robots
    
    % Limit acceleration and velocities
    for i = 1:n
        % If velocities are >= max allowable, don't accelerate
        if(abs(X(3,i)) >= vmax)
            dX(3,i) = 0;
        end
        
        if(abs(X(4,i)) >= vmax)
            dX(4,i) = 0;
        end
        
        % Normalize accelerations
        anorm = norm(dX(3:4,i));
        if anorm > amax
            dX(3:4,i) = dX(3:4,i)./(anorm/amax);
        end
    end
    
    % Reshape into a vector 
    dx = dX(:);
end

% Force on pursuer at x1 given x1 is a pursuer, x2 is another pursuer
function [force] = pursuer_pursuer_force(x1, x2)
    r = x2 - x1;
    k = 0.2; % tune this
    force = -k/norm(r)*r;
end

% Force on pursuer at x1 given x1 is a pursuer, x2 is an evader
function [force] = pursuer_evader_force(x1, x2)
    r = x2 - x1;
    k = 4; % tune this
    force = k/norm(r)*r;
end

% Force on evader at x1 given x1 is an evader, x2 is a pursuer
function [force] = evader_pursuer_force(x1, x2)
    r = x2 - x1;
    k = 0.5; % tune this
    
    % Evader can only see neighbors in a certain distance
    if norm(r) <= 3
        force = -k/norm(r)*r;
    else
        force = 0;
    end
end

% Force on evader at x1 given x1 is an evader, x2 is an evader
function [force] = evader_evader_force(x1, x2)
    r = x2 - x1;
    k = 0.1; % tune this
    
    % Evader can only see neighbors in a certain distance
    if norm(r) <= 3
        force = -k/norm(r)*r;
    else
        force = 0;
    end
end
