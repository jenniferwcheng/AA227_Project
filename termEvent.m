function [value, isterminal, direction] = termEvent(t, x)
    % Terminates ode call when all evaders are caught
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
    
    
    % For one evader
%     % Extract evader and pursuer locations
%     px = x(5:4:end-1);
%     py = x(6:4:end);
%     ex = x(1)*ones(length(px),1);
%     ey = x(2)*ones(length(py),1);
%     capture_radius = 0.2; % [m]
%     
%     % Evaluate distance from each pursuer to evader
%     dist_to_evader = [ex ey] - [px py];
%     
%     % Normalize the distance
%     d_norm = vecnorm(dist_to_evader,2,2); 
    
%     value      = (sum(d_norm <= capture_radius) == 0); % Check if any of the pursuers are within capture radius
    
    value = (sum(caught) ~= ne); % Stop integration when all evaders caught, terminates when value == 0
    isterminal = 1;   % Stop the integration
    direction  = 0;
end