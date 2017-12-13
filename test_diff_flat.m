
%% Add paths
user = 'nathan';
% addpath(genpath([pwd, '/@Quadrotorload']));
addpath(genpath([pwd, '/controllers/']));
addpath(genpath([pwd, '/gen/']));

addpath(genpath([pwd, '/traj_gen/']));

%% Reset workspace
clear
clc
close all

%% Build quadrotor system
params = struct;
sys = Quadrotor(params);
sys.controller = @controller2;

%% testing reference
t = 0:0.1:100;
xREFERENCE = [];
for i = 1:length(t)
    x_ = generate_ref_trajectory(t(i)  ,sys);
    xREFERENCE = [xREFERENCE, x_];
end
plot(xREFERENCE(1,:),xREFERENCE(2,:));
    
%% Simulate System
solver = @ode45;
tspan = [0,10];
x0 = generate_ref_trajectory(0, sys);
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
opts.t = time;
opts.x = states;
opts.vid.MAKE_MOVIE = false;
opts.vid.filename = './results/vid1';
sys.animateQuadrotor(opts);

