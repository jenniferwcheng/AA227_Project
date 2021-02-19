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

