%% 
% Code to implement MPC for trajectory tracking of quadrotor position

%% Setting up env
addpath(genpath([pwd, '/controllers/']));
addpath(genpath([pwd, '/gen/']));

%% Reset workspace
clear
clc
close all
yalmip('clear')

%% Build quadrotor system
params = struct;
sys = Quadrotor(params);

% input bounds
sys.Fmin = 0;
sys.Fmax = 2*sys.mQ*sys.g; % --> *2bUpdated*

%% Load reference trajectory
xref = zeros(sys.nDof,1);
uref = [sys.mQ*sys.g/2;
    sys.mQ*sys.g/2];

%% Initial condition
x0 = [-.5;-.5;0;0;0;0];

%% MPC 
% params
params.mpc.Tf = 10;
params.mpc.Ts = .1;
params.mpc.M = params.mpc.Tf/params.mpc.Ts;
params.mpc.N = 10;

params.mpc.Q = 100*eye(sys.nDof);
params.mpc.R = 1*eye(sys.nAct);
params.mpc.P = params.mpc.Q;    

% system response
sys_resp.x = zeros(sys.nDof,params.mpc.M+1);
sys_resp.u = zeros(sys.nAct,params.mpc.M);
sys_resp.x(:,1) = x0;

% calculating input over the loop
for impc = 1:params.mpc.M
    fprintf('loop %d\n',impc);
    
    % optimizing for input
    xk = sys_resp.x(:,impc);
    ctlx = solve_for_cftoc(xk,xref,uref,sys,params);
    
    % forward simulation
    [A,B] = sys.discretizeLinearizeQuadrotor(params.mpc.Ts, xref,uref);
    u = ctlx.uOpt(:,1);
    sys_resp.x(:,impc+1) = A*xk+B*u;
    sys_resp.u(:,impc) = u;
end


%% plots
time = 0:params.mpc.Ts:params.mpc.Tf;
figure
plot(time', sys_resp.x');
legend('y','z', 'phi', 'dy', 'dz', 'dphi');
xlabel('time (s)');
ylabel('states');
grid on;

figure
plot(time(1:end-1), sys_resp.u);
legend('F_1', 'F_2');
xlabel('time (s)');
ylabel('inputs');
grid on;

%% Animate
opts.t = time';
opts.x = sys_resp.x';
opts.vid.MAKE_MOVIE = false;
sys.animateQuadrotor(opts);





