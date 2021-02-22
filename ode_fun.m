function [capture_time] = start_sim(num_evaders, num_pursuers)
     x0 = rand(num_evaders*4+num_pursuers*4,1);
     [t, x] = ode23(@ode_fun,x0)
     capture_time = t(end)
end

function dx = ode_fun(t, x)
    % Shape the state and derivative into
    % dims x num_robots matrices
    
    dims = 4
    n = length(x)/dims
    num_pursuers = n - 1;
    num_evaders = 1;
   
    X = reshape(x, dims, n)
    forces = zeros(dims/2, n)
    
    % Calculate the force for the ith robot
    for i = 1:n
        x1 = X(i, :)
        % Account for force from any other robot
        for j = 1:n
            x2 = X(j, :)
            
            % Consider each poossible evader, pursuer pairing
            if i <= num_evaders
                % Evader, evader
                if j <= num_evaders
                    forces(i, :) = forces(i, :) + evader_evader_force(x1, x2)
                % Evader, pursuer
                else 
                    forces(i, :) = forces(i, :) + evader_pursuer_force(x1, x2)
                end            
            else
                % Pursuer, evader
                if j <= num_evaders
                    forces(i, :) = forces(i , :) + pursuer_evader_force(x1, x2)
                % Pursuer, pursuer
                else
                    forces(i, :) = forces(i, :) + pursuer_pursuer_force(x1, x2)
                end
            end
        end
    end
    
    % Return the vectorized version of the derivative
    dX = zeros(dims, n)
    dX(1:2, :) = X(3:4, :) % Change position by the velocity
    dX(3:4, :) = forces % Change velocity by the force, assume m = 1 for all robots
    
    % Reshape into a vector 
    dx = dX(:)
end

% 
function dx = ode_fun(t,x)   
    % x1,2 x1,2 x2,1 x2,2 ... xn,1 xn,2
    n = length(x)/2;
    dx = zeros(n*2,1);    
    
    kp = 10;
    ke = 2;
    
    % evader derivative
    for j = 2:n
        xi = [x(1); x(2)]; % evader
        xj = [x(2*j-1); x(2*j)]; % pursuer
        r = xj - xi;

        dx(1:2) = dx(1:2) + ke*1/norm(r)^3*r; % dv = F/m*dt; m = 1, dt = ?
    end
                 
    % calculate derivatives for pursuers
    for i = 2:n
        xj = [x(1); x(2)]; % evader
        xi = [x(2*i-1); x(2*i)]; % pursuer
        r = xi - xj;
       
        if norm(r) > 0.5 % pursuer larger than range
            dx(2*i-1:2*i) = -kp*1/norm(r)^3*r;
            
        else % caught the evader
            dx(end) = 1;
        end      
    end
    
%     plotx = x(1:2:end);
%     ploty = x(2:2:end);
%     plot(plotx,ploty, '.', 'MarkerSize', 20)
%     xlim([-1 15])
%     ylim([-1 15])
%     drawnow
end

% Force on pursuer at x1 given x1 is a pursuer, x2 is another pursuer
function pursuer_pursuer_force(x1, x2)
    
end

% Force on pursuer at x1 given x1 is a pursuer, x2 is an evader
function pursuer_evader_force(x1, x2)

end

% Force on evader at x1 given x1 is an evader, x2 is a pursuer
function evader_pursuer_force(x1, x2)

end

% Force on evader at x1 given x1 is an evader, x2 is an evader
function evader_evader_force(x1, x2)
end