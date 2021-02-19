function [value, isterminal, direction] = termEvent(t, x)
    value      = (x(end) - 1);
    isterminal = 1;   % Stop the integration
    direction  = 0;
end