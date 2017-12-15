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
% folder = './results/Testcase_1_keyhole_2m/';
folder = './results/Testcase_2_inverted_pendulum/';
% folder = './results/Testcase_3_triangles/';
filename = 'workspace.mat';

DATA = load(strcat(folder,filename));
xref = DATA.traj.x;
uref = DATA.traj.u;
tref = DATA.traj.t;

N = 3;

xref = [xref, repmat(xref(:,end),1,N)];
uref = [uref, repmat(uref(:,end),1,N)];

%% Build quadrotor system
params = struct;
% input bounds
bounds.inputs.lb = [0; 0];
bounds.inputs.ub = [20;20];%[15;15]; 
% state bounds 
bounds.states.lb = DATA.xL;
bounds.states.ub = DATA.xU;
sys = Quadrotorload(params,bounds);

%% acutal model
p.mL = 0.5;
act_sys = Quadrotorload(p);


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
% params.mpc.Q = diag([100,100,10,10,1,1,1,1]);
params.mpc.Q = 1*eye(sys.nDof);
params.mpc.R = .1*eye(sys.nAct);
params.mpc.P = params.mpc.Q;    

sys.controlParams = params;

%% MPC Control 
[mpc_response] = sys.mpc_load_Tracking(x0,tref,xref,uref,'DNL',act_sys);

%% DLQR Control 
[dlqr_response] = sys.dlqr_load_Tracking(x0,tref,xref,uref,'CNL',act_sys);

%% saving 
save(strcat(folder,'control'),'dlqr_response','mpc_response');

%% plots
xref = DATA.traj.x;
uref = DATA.traj.u;
    
% trajectory
fig1 = figure; hold on;
l_ref = plot(xref(1,:),xref(2,:),'k','linewidth',1);
l_mpc = plot(mpc_response.x(1,:),mpc_response.x(2,:),'g','linewidth',2);
l_dlqr = plot(dlqr_response.x(1,:),dlqr_response.x(2,:),'--r','linewidth',2);
for i = 1:length(DATA.O)
    plot(DATA.O{i},'alpha',0.5); hold on;
end
leg = latex_legend({'REFERENCE','MPC','DLQR','OBSTACLES'});
leg.AutoUpdate = 'off';
latex_title('TRAJECTORY');
latex_xlabel('y [m]');latex_ylabel('z [m]');
drawQuadrotorload(mpc_response.x(:,1),sys.l);
drawQuadrotorload(mpc_response.x(:,end),sys.l);

% inputs
fig2 = figure; 
subplot(2,1,1);hold on;
l_ref = plot(tref(1:end-1),uref(1,:),'-k','linewidth',1);
l_mpc = plot(mpc_response.t(1:end-1),mpc_response.u(1,:),'g','linewidth',2);
l_dlqr = plot(dlqr_response.t(1:end-1),dlqr_response.u(1,:),'--r','linewidth',2);
leg = latex_legend({'REFERENCE','MPC','DLQR'});
leg.AutoUpdate = 'off';
latex_title('F1');grid on; grid minor;
latex_xlabel('t [s]');latex_ylabel('$F_1$ [N]');
subplot(2,1,2);hold on;
l_ref = plot(tref(1:end-1),uref(2,:),'-k','linewidth',1);
l_mpc = plot(mpc_response.t(1:end-1),mpc_response.u(2,:),'g','linewidth',2);
l_dlqr = plot(dlqr_response.t(1:end-1),dlqr_response.u(2,:),'--r','linewidth',2);
leg = latex_legend({'REFERENCE','MPC','DLQR'});
leg.AutoUpdate = 'off';
latex_title('F2');grid on; grid minor;
latex_xlabel('t [s]');latex_ylabel('$F_2$ [N]');

time = mpc_response.t;
f1 = figure;
subplot(2,4,1);
plot(time', mpc_response.x(1,:)','r'); hold on;
plot(tref',xref(1,:)','*b');legend('y','y_{ref}');
title('y');
xlabel('time (s)');
ylabel('m');
grid on; grid minor;
subplot(2,4,2);
plot(time', mpc_response.x(2,:)','r');hold on;
plot(tref',xref(2,:)','*b');legend('z','z_{ref}');
title('z');
xlabel('time (s)');
ylabel('m');
grid on; grid minor;
subplot(2,4,3);
plot(time', (180/pi)*mpc_response.x(3,:)','r');hold on;
plot(tref',(180/pi)*xref(3,:)','*b');legend('\phi_L','{\phi_L}_{ref}');
title('phi_L');
xlabel('time (s)');
ylabel('degrees'); grid on; grid minor;
subplot(2,4,4);
plot(time', (180/pi)*mpc_response.x(4,:)','r');hold on;
plot(tref',(180/pi)*xref(4,:)','*b');legend('\phi_Q','{\phi_Q}_{ref}');
title('phi_Q');
xlabel('time (s)');
ylabel('degrees');
grid on; grid minor;

subplot(2,4,5);
plot(time', mpc_response.x(5,:)','r');hold on;
plot(tref',xref(5,:)','*b'); legend('dy','dy_{ref}');
title('dy');
xlabel('time (s)');
ylabel('m/s');
grid on; grid minor;
subplot(2,4,6);
plot(time', mpc_response.x(6,:)','r');hold on;
plot(tref',xref(6,:)','*b'); legend('dz','dz_{ref}');
title('dz');
xlabel('time (s)');
ylabel('m/s');
grid on; grid minor;
subplot(2,4,7);
plot(time', mpc_response.x(7,:)','r');hold on;
plot(tref',xref(7,:)','*b'); legend('dphiL','dphiL_{ref}');
title('dphi_L');
xlabel('time (s)');
ylabel('rad/s');
grid on; grid minor;
subplot(2,4,8);
plot(time', mpc_response.x(8,:)','r');hold on;
plot(tref',xref(8,:)','*b'); legend('dphiQ','dphiQ_{ref}');
title('dphi_Q');
xlabel('time (s)');
ylabel('rad/s');
grid on; grid minor;

figure;
plot(mpc_response.x(1,:),mpc_response.x(2,:),'r','linewidth',2);hold on;
plot(xref(1,:),xref(2,:),'*b','linewidth',2);
legend('x','xref');
grid on; grid minor;
xlabel('Y');ylabel('Z');
title('output trajectory');

figure
plot(time(1:end-1), mpc_response.u(1,:),'r','linewidth',2);hold on;
plot(tref(1:end-1), uref(1,:),'b','linewidth',2);
legend('F_1', 'F_1 ref');
xlabel('time (s)');
ylabel('inputs');
grid on; grid minor;

keyboard;
%% Animate
opts.t = mpc_response.t;
opts.x = mpc_response.x';
opts.td = tref';
opts.xd = xref';
opts.O = DATA.O;

opts.vid.MAKE_MOVIE = true;
opts.vid.filename  = strcat(folder,'video');
sys.animateQuadrotorload(opts);





