
%% Add paths
user = 'ayush';
% addpath(genpath([pwd, '/@Quadrotor']));
addpath(genpath([pwd, '/controllers/', user]));
addpath(genpath([pwd, '/gen/']));

%% reset
clear
clc

%% Build quadrotor system
params = struct;
sys = Quadrotor(params);
sys.controller = @controller;

%% Simulate System
solver = @ode45;
tspan = [0,5];
x0 = [-10.5;-10.5;0;0;0;0];
sol = sys.simulate(tspan, x0, solver);
%% Plot

states = sol.y;
time = sol.x;

% compute inputs:
control = [];
for i = 1:length(time)
    control(:,i) = sys.calcControlInput(time(i), states(:,i));
end

figure
plot(time, states);
legend('y','z', '\phi', 'dy', 'dz', 'dphi');
xlabel('time (s)');
ylabel('states');
grid on;

figure
plot(time, control);
legend('F_1', 'F_2');
xlabel('time (s)');
ylabel('inputs');
grid on;

%% Animate
sys.animateQuadrotor(time,states,[0;0]);