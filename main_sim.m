
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
sys.controller = @controller_dlqr_path;


%% Generate obstacle avoiding trajectory
traj = traj_gen_QRL_polyhedron(sys);
%% Generate trajectory to track
load traj_gen_QRL
time = traj.t;
states = traj.x;
control = traj.u;
sys.controlParams = struct('time',time,'states',states,'control',control);


%% Simulate System
solver = @ode45;
tspan = [0,10];
x0 = states(:,1);
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
sys.animateQuadrotorload(opts);

