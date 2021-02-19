%% potential field method
clear all; close all; clc;

% num of robots
n = 5;
       
tspan = [0 100];
x0 = [0; 0; -4; -4; 4; 4; 4.5; -4; -4; 4; 0]; 
% x0 = 5*rand([n*2,1]);
% x0 = [x0; 0];

Opt = odeset('Events', @termEvent);
[t, x] = ode23(@ode_fun,tspan, x0, Opt);

% plot with trajectories

figure % top down view
% plot each robot vs time
for i = 1:n
    if i == 1
        plot(x(:,2*i-1), x(:,2*i), 'r', 'MarkerSize', 10)
        hold on
        plot(x(end,2*i-1), x(end,2*i), 'xr', 'MarkerSize', 10)
        plot(x(1,2*i-1), x(1,2*i), '.r', 'MarkerSize', 10)
    else
        plot(x(1,2*i-1), x(1,2*i), '.g', 'MarkerSize', 10)
        plot(x(:,2*i-1), x(:,2*i), '--b', 'MarkerSize', 10)
        plot(x(end,2*i-1), x(end,2*i), 'xg', 'MarkerSize', 10)
    end
end

grid on

% add legend and title
title('Trajectories')
% legend('Evader', 'Pursuer 1', 'Pursuer 2')
xlabel('x1')
ylabel('x2')

%% plotting against time
figure
subplot(2,1,1) % x1 state
for i = 1:n
    plot(t, x(:,2*i-1))
    hold on
end
grid on

% legend('Robot 1', 'Robot 2', 'Robot 3', 'Robot 4', 'Robot 5', 'Robot 6')
title('x1')

xlabel('t')
ylabel('x1')

subplot(2,1,2) % x2 state
for i = 1:n
    plot(t, x(:,2*i))
    hold on
end
grid on

legend('Robot 1', 'Robot 2', 'Robot 3', 'Robot 4', 'Robot 5', 'Robot 6')
title('x2')

xlabel('t')
ylabel('x2')