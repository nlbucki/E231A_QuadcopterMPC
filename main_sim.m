
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
%% Generate Quadrotor Polyhedron
QR_width = sys.wQ;
QR_height = sys.hQ;
load_r = sys.wL;
QR = Polyhedron('V',[-QR_width/2 0; QR_width/2 0; -QR_width/2 QR_height;...
    QR_width/2 QR_height]);
L = Polyhedron('V',[-load_r -load_r; load_r -load_r; -load_r sys.l; ...
    load_r sys.l]);
%% Generate Obstacle
O ={Polyhedron('V',[-2 1; -20 1; -2 -2; -20 -2])};
%% Generate obstacle avoiding trajectory
%traj = traj_gen_QRL_polyhedron(sys);
%% Generate trajectory to track
load traj_gen_QRL
time_traj = traj.t;
states_traj = traj.x;
control_traj = traj.u;
sys.controlParams = struct('time',time_traj,'states',states_traj,'control',control_traj);
%% Assign Controller
% Create Copies of Object
k=1; %number of controllers
Quad=cell(1,k);
for i=1:1
    Quad{i}=sys;
end
Quad{1}.controller = @controller_dlqr_path;
% Quad{2}.controller = @controller_dlqr_path_SensorNoise;
% Quad{3}.controller = @controller_dlqr_path_InputDisturbance;
%% Simulate System
solver = @ode45;
tspan = [0,10];
x0 = states_traj(:,1);
sol=cell(1,k);
states=cell(1,k);
control=cell(1,k);
time=cell(1,k);
for i=1:k
  sol{i} = Quad{i}.simulate(tspan, x0, solver);
  states{i} = sol{i}.y;
  time{i} = sol{i}.x;
  c = zeros(2,length(time{i}));
    for j = 1:length(time{i})
        c(:,j) = Quad{i}.calcControlInput(time{i}(j), states{i}(:,j));
    end
    control{i}=c;
end
%% Plot States and Control Inputs

figure
plot(time{1}, states{1});
legend('y','z', '\phi', 'dy', 'dz', 'dphi');
xlabel('time (s)');
ylabel('states');
grid on;

figure
plot(time{1}, control{1});
legend('F_1', 'F_2');
xlabel('time (s)');
ylabel('inputs');
grid on;
%% Plot Quadrotor Polyhedron with desired Trajectory
color='g'; %Color of Trajectory
figure;
hold on;
plot(traj.x(1,:),traj.x(2,:),'x-');
plotQuadcopterPolyhedron(states{1},QR,L,Quad{1},O,color);
legend('Generated Trajectory', 'True Trajectory');
%% 
% figure;
% x_interp=interp1(time,states(1,:),traj.t);
% y_interp=interp1(time,states(2,:),traj.t);
% figure;
% hold on;
% plot(traj.t,x_interp-traj.x(1,:))
% plot(traj.t,y_interp-traj.x(2,:),'r');
% legend('\delta x', '\delta y');
% title('Derivation from trajectory');
% 
% figure
% hold on;
% plot(traj.x(1,:),traj.x(2,:),'x-')
% xlabel('y')
% ylabel('z')
% plot(states(1,:),states(2,:),'x-');
% legend('Generated path with obstacle avoidance','True Path tracked by LQR');
% hold on
% 
%% Animate
opts.t = time{1};
opts.x = states{1};
opts.vid.MAKE_MOVIE = false;
opts.vid.filename = './results/vid1';
sys.animateQuadrotorload(opts);

