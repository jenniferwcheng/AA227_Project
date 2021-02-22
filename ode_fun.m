% function [capture_time] = start_sim(num_evaders, num_pursuers)
%      x0 = rand(num_evaders*4+num_pursuers*4,1);
%      [t, x] = ode23(@ode_fun,x0)
%      capture_time = t(end)
% end

function dx = ode_fun(t, x)
    x = x(1:end-1); % remove the flag
    caught = 0; % indicates if any pursuer is in capture radius of evader
    
    % Shape the state and derivative into
    % dims x num_robots matrices
    dims = 4;
    n = length(x)/dims;
    num_pursuers = n - 1;
    num_evaders = 1; % hard coded 1 evader for now
    
    % TODO: define velocity and accel maximums
    vmax = 2; % tune these
    amax = 10;
   
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
                        r = norm(x2-x1);
                        if r <= 0.1 % check if pursuer is within capture radius
                            caught = 1;
                        end
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
    
    % TODO: Limit acceleration and velocities
    for i = 1:n
        % check velocities
        vnorm = norm(dX(1:2,i));
        if vnorm > vmax
            dX(1:2,i) = dX(1:2,i)./(vnorm/vmax);
        end
        
        % check accelerations
        anorm = norm(dX(3:4,i));
        if anorm > amax
            dX(3:4,i) = dX(3:4,i)./(anorm/amax);
        end
    end
    
    % Reshape into a vector 
    dx = dX(:);
    dx = [dx; caught];
    
    % plot in real time
    plot(x(1), x(2), '.r', 'MarkerSize', 20)
    hold on
    plotx = x(5:4:end);
    ploty = x(6:4:end);
    plot(plotx,ploty, '.b', 'MarkerSize', 20)
    xlim([-50 50])
    ylim([-50 50])
    drawnow
    hold off
end

% Force on pursuer at x1 given x1 is a pursuer, x2 is another pursuer
function [force] = pursuer_pursuer_force(x1, x2)
    r = x2 - x1;
    k = 0.1; % tune this
    force = -k/norm(r)*r;
end

% Force on pursuer at x1 given x1 is a pursuer, x2 is an evader
function [force] = pursuer_evader_force(x1, x2)
    r = x2 - x1;
    k = 3; % tune this
    force = k/norm(r)*r;
end

% Force on evader at x1 given x1 is an evader, x2 is a pursuer
function [force] = evader_pursuer_force(x1, x2)
    r = x2 - x1;
    k = 1; % tune this
    force = -k/norm(r)*r;
end

% Force on evader at x1 given x1 is an evader, x2 is an evader
function [force] = evader_evader_force(x1, x2)
    r = x2 - x1;
    k = 1; % tune this
    force = -k/norm(r)*r;
end
