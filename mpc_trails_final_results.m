%% 
% Code to implement MPC for trajectory tracking of quadrotor position

%% Setting up env
addpath(genpath([pwd, '/controllers/']));
addpath(genpath([pwd, '/gen/']));

%% Reset workspace
clear
clc
static_disp('');
close all
yalmip('clear')


%% Build quadrotor system
params = struct;
sys = Quadrotorload(params);

%% Load reference trajectory
% diff-flat trajectory
% --------------------
% tref = [0:params.mpc.Ts:params.mpc.Tf];
% xref = [];
% uref = [];
% for i = 1:length(tref)
% %     [x_, u_] = generate_ref_trajectory(tref(i)  ,sys);
%     x_ = zeros(sys.nDof,1);
%     u_ = (sys.mQ)*sys.g*0.5*ones(sys.nAct,1);
%     xref = [xref,x_];
%     uref = [uref,u_];
% end

%% load optimized results
load('./results/traj_load_inverted_pendelum.mat');
xref = traj.x;
uref = traj.u;
tref = traj.t;

N = 3;

xref = [xref, repmat(xref(:,end),1,N)];
uref = [uref, repmat(uref(:,end),1,N)];

%% Initial condition
% x0 = [-.5;-.5;0;0;0;0];
x0  = xref(:,1);

%% MPC params
% params
% params.mpc.Tf = 10;
% params.mpc.Ts = .1;
% params.mpc.M = params.mpc.Tf/params.mpc.Ts;
% params.mpc.N = 10;

params.mpc.M = length(tref)-1;
params.mpc.N = N;

% gains
params.mpc.Q = diag([100,100,10,10,1,1,1,1]);
% params.mpc.Q = eye(sys.nDof);
params.mpc.R = 1*eye(sys.nAct);
params.mpc.P = params.mpc.Q;    

sys.controlParams = params;


%% Control 
[sys_response] = sys.mpc_load_Tracking(x0,tref,xref,uref,'DL');

%% plots
time = sys_response.t;
figure
subplot(2,3,1);
plot(time', sys_response.x(1,:)');
title('y');
xlabel('time (s)');
ylabel('m');
grid on; grid minor;
subplot(2,3,2);
plot(time', sys_response.x(2,:)');
title('z');
xlabel('time (s)');
ylabel('m');
grid on; grid minor;
subplot(2,3,3);
plot(time', (180/pi)*sys_response.x(3,:)');
title('phi');
xlabel('time (s)');
ylabel('degrees');
grid on; grid minor;
subplot(2,3,4);
plot(time', sys_response.x(4,:)');
title('dy');
xlabel('time (s)');
ylabel('m/s');
grid on; grid minor;
subplot(2,3,5);
plot(time', sys_response.x(5,:)');
title('dz');
xlabel('time (s)');
ylabel('m/s');
grid on; grid minor;
subplot(2,3,6);
plot(time', sys_response.x(6,:)');
title('dphi');
xlabel('time (s)');
ylabel('rad/s');
grid on; grid minor;


figure;
plot(sys_response.x(1,:),sys_response.x(2,:),'r','linewidth',2);hold on;
plot(xref(1,:),xref(2,:),'b','linewidth',2);
legend('x','xref');
grid on; grid minor;
xlabel('Y');ylabel('Z');
title('output trajectory');

figure
plot(time(1:end-1), sys_response.u);
legend('F_1', 'F_2');
xlabel('time (s)');
ylabel('inputs');
grid on; grid minor;

keyboard;
%% Animate
opts.t = sys_response.t;
opts.x = sys_response.x';
opts.td = time';
opts.xd = xref(:,1:params.mpc.M+1)';
opts.vid.MAKE_MOVIE = false;
sys.animateQuadrotorload(opts);





