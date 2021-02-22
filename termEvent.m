function [value, isterminal, direction] = termEvent(t, x)
    value      = (x(end) == 0); % check if the last element is non-zero (means captured)
    isterminal = 1;   % Stop the integration
    direction  = 0;
end