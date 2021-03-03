%% Simulate pursuit-evasion
clear all; close all; clc;

%----Parameters-----------
ne = 1; % Number of evaders
np = 3; % Number of pursuers
n = ne + np; % Total number of robots
dim = 4; % Order of states

capture_radius = 0.2; % [m] (If changing this value, remember to change in termEvent.m as well)
vmax = 1; % [m/s] Same for both methods? 
amax = 10; % [m/s^2] 
grid_size = 20; % [m] size of environment (length) -> area = grid_size^2
global F; % For video

% Flags
method = 1; % 0 for potential, 1 for Voronoi
save_video = 0; % 1 to plot in real time and save video
monte_carlo = 0; % 1 - on, 0 - off

tend = 10; % [s] length of simulation time

% Monte Carlo params
MAX_ITERS = 250; % Iterations for Monte Carlo
success_rate = 0; % Number of successes/MAX_ITERS
average_capture_time = 0; % Total capture time/MAX_ITERS

%----Initial conditions------

% Random positions:
x0 = grid_size/2*rand([n*dim,1]) - grid_size/4;
x0(3:dim:end) = 0; % zero velocity
x0(4:dim:end) = 0; % zero acceleration

% x0 = [0; 0; 0; 0; -4; -4; 0; 0; 4; 4; 0; 0; 5; -4; 0; 0; -4; 4; 0; 0]; % square

%-------Run ODE function---------------
Opt = odeset('Events', @termEvent); % Terminate when within capture radius
% [t, x] = ode23(@ode_fun,tspan, x0, Opt);
if monte_carlo
    for i = 1:MAX_ITERS
        % Random positions:
        x0 = grid_size/4*rand([n*dim,1]) - grid_size/8;
        x0(3:dim:end) = 0; % zero velocity
        x0(4:dim:end) = 0; % zero acceleration

        [t, x] = ode23(@(t,x) ode_fun(t,x, method, save_video, vmax, amax, ne, np, grid_size),[0 tend], x0, Opt);
        if t(end) < tend
            success_rate = success_rate + 1;
            average_capture_time = average_capture_time + t(end);
        end
    end
    average_capture_time = average_capture_time/MAX_ITERS;
    success_rate = success_rate/MAX_ITERS;
    
    fprintf('Number of pursuers: %0.5g \n', np)
    fprintf('Number of evaders: %0.5g \n', ne)
    fprintf('Capture radius: %0.5g \n', capture_radius)
    fprintf('Number of iterations: %0.5g \n', MAX_ITERS)
    fprintf('Success rate: %0.5g \n', success_rate)
    fprintf('Average capture time: %0.5g \n', average_capture_time)
else
    [t, x] = ode23(@(t,x) ode_fun(t,x, method, save_video, vmax, amax, ne, np, grid_size),[0 tend], x0, Opt);

    %-------Video--------------------------
%     if save_video % Save video
%         writerObj = VideoWriter('myVideo.avi');
%         writerObj.FrameRate = 10;
%         open(writerObj);
%         % write the frames to the video
%         for i=1:length(F)
%             % convert the image to a frame
%             frame = F(i) ;    
%             writeVideo(writerObj, frame);
%         end
%         close(writerObj);
%     end

    %-------Determine capture time-----------
    capture_time = t(end);
    if capture_time < tend
        fprintf('Number of pursuers: %0.5g \n', np)
        fprintf('Number of evaders: %0.5g \n', ne)
        fprintf('Capture radius: %0.5g \n', capture_radius)
        fprintf('Capture time: %0.5g \n', capture_time)
        tend = capture_time;
    else
        fprintf('Number of pursuers: %0.5g \n', np)
        fprintf('Number of evaders: %0.5g \n', ne)
        fprintf('Capture radius: %0.5g \n', capture_radius)
        disp("Capture not successful")
    end

    %-------Trajectory plot-----------------
    close all;
    set(0,'DefaultFigureWindowStyle','docked')
    figure
    for i = 1:n
        if i == 1 % Evader
            plot(x(:,4*i-3), x(:,4*i-2), 'r', 'MarkerSize', 10) % Trajectory 
            hold on
            plot(x(1,4*i-3), x(1,4*i-2), '.r', 'MarkerSize', 10) % Initial position
            plot(x(end,4*i-3), x(end,4*i-2), 'xr', 'MarkerSize', 10) % Final position
        else % Pursuer 
            plot(x(:,4*i-3), x(:,4*i-2), '--b', 'MarkerSize', 10) % Trajectory
            plot(x(1,4*i-3), x(1,4*i-2), '.b', 'MarkerSize', 10) % Initial position
            plot(x(end,4*i-3), x(end,4*i-2), 'xb', 'MarkerSize', 10) % Final position
        end
    end
    grid on
    title('Trajectories')
    xlabel('x1')
    ylabel('x2')
    axis equal
    xlim([-grid_size/2 grid_size/2])
    ylim([-grid_size/2 grid_size/2])

    %----------Plot distance to evader-----------

    figure % Distance to evader 
    for i = 2:n
        plot(t, vecnorm(x(:,1:2) - x(:,4*i-3:4*i-2),2,2))
        hold on
    end
    plot(t, capture_radius*ones(length(t)), 'k--')
    grid on

    title('Distance to Evader')

    xlabel('t')
    ylabel('d')

    %----------Plot velocities----------------
    figure
    subplot(2,1,1) % x velocity
    for i = 1:n
        plot(t, x(:,4*i-1))
        hold on
    end
    plot(t, vmax*ones(length(t)),'k--', t, -vmax*ones(length(t)), 'k--') % min max lines
    title('X Velocities')
    xlabel('t')
    ylabel('Vx')
    grid on
    ylim(2*[-vmax,vmax])
    xlim([0,tend])

    subplot(2,1,2) % y velocity
    for i = 1:n
        plot(t, x(:,4*i))
        hold on
    end
    plot(t, vmax*ones(length(t)), 'k--', t, -vmax*ones(length(t)), 'k--') % min max line
    grid on
    title('Y Velocities')
    xlabel('t')
    ylabel('Vy')
    ylim(2*[-vmax,vmax])
    xlim([0,tend])
end