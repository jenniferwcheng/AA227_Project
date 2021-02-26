function [value, isterminal, direction] = termEvent(t, x)
    % Extract evader and pursuer locations
    px = x(5:4:end-1);
    py = x(6:4:end);
    ex = x(1)*ones(length(px),1);
    ey = x(2)*ones(length(py),1);
    capture_radius = 0.2; % [m]
    
    % Evaluate distance from each pursuer to evader
    dist_to_evader = [ex ey] - [px py];
    
    % Normalize the distance
    d_norm = vecnorm(dist_to_evader,2,2); 
    
    value      = (sum(d_norm <= capture_radius) == 0); % Check if any of the pursuers are within capture radius
    isterminal = 1;   % Stop the integration
    direction  = 0;
end