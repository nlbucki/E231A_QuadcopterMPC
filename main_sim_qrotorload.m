
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
sys = Quadrotorload(params);
sys.controller = @controller;


%% Generate obstacle avoiding trajectory
traj = traj_gen_QRL_polyhedron(sys);
%% Generate trajectory to track
% load trajectory
% time = traj.t;
% states = traj.x;
% control = traj.u;
% sys.controlParams = struct('time',time,'states',states,'control',control);


%% Simulate System
solver = @ode45;
tspan = [0,10];
x0 = [-10.5;-10.5;0;0;0;0;0;0];
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
subplot(2,2,1);
plot(time, states(1,:),'r',time, states(2,:),'b');
legend('y','z');
xlabel('time (s)');
ylabel('states');
grid on;
subplot(2,2,2);
plot(time, states(3,:),'r',time, states(4,:),'b');
legend('\phi_L','\phi_Q');
xlabel('time (s)');
ylabel('states');
grid on;
subplot(2,2,3);
plot(time, states(5,:),'r',time, states(6,:),'b');
legend('dy','dz');
xlabel('time (s)');
ylabel('states');
grid on;
subplot(2,2,4);
plot(time, states(7,:),'r',time, states(8,:),'b');
legend('d\phi_L','d\phi_Q');
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
sys.animateQuadrotorload(opts);

