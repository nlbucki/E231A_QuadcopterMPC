%% Setting up env
addpath(genpath([pwd, '/controllers/']));
addpath(genpath([pwd, '/gen/']));
%% Reset workspace
clear
clc
static_disp('');
close all
yalmip('clear')
%% load optimized results
% folder = './results/Testcase_1_keyhole_2m/';
% folder = './results/Testcase_2_inverted_pendulum/';
folder = './results/Testcase_3_triangles/';
filename = 'workspace.mat';

DATA = load(strcat(folder,filename));
xref = DATA.traj.x;
uref = DATA.traj.u;
uref(:, end+1) = uref(:, end);
tref = DATA.traj.t;
tdiff = diff(tref);
% tdiff = tdiff;
TrefNew = zeros(1, 2*length(tref)-1);
TrefNew(1:2:end) = tref;
TrefNew(2:2:end-1) = tref(1:end-1) + tdiff(1)/2;
%% Build quadrotor system
params = struct;
% input bounds
bounds.inputs.lb = [0; 0];
bounds.inputs.ub = 1*[15;15]; 
% state bounds 
bounds.states.lb = DATA.xL;
bounds.states.ub = DATA.xU;
sys = Quadrotorload(params,bounds);
nx = sys.nDof;
nu = sys.nAct;


fref = zeros(nx, length(uref));

for i = 1:length(fref)
    fref(:, i) = sys.systemDynamics(tref(i), xref(:, i), uref(:, i));
end

xrefNew = zeros(nx, length(TrefNew));

for i = 1:length(xrefNew)
    xrefNew(:, i) = bSpline2(tref,xref,fref,TrefNew(i));
end

urefNew = zeros(nu, length(TrefNew));
urefNew(1, :) = interp1(tref(1:end), uref(1, :), TrefNew, 'linear');
urefNew(2, :) = interp1(tref(1:end), uref(2, :), TrefNew, 'linear');
xrefOld = xref;
trefOld = tref;
urefOld = uref;

xref = xrefNew;
tref = TrefNew;
uref = urefNew;

N = 10;
xref = [xref, repmat(xref(:,end),1,N)];
uref = [uref, repmat(uref(:,end),1,N)];


%% MPC control

Q = 1*eye(nx);
P = Q;
R = eye(nu);
M = length(tref)-1; % no. of sim steps
tref = [tref, repmat(tref(:,end),1,N)];
%% Initial condition
% x0 = [-.5;-.5;0;0;0;0];
xk  = xref(:,1);

Ts = diff(tref);
Ts = Ts(1);
x = sdpvar(nx, N+1);
u = sdpvar(nu, N);
%% simulate sys
tstart = 0;
options = sdpsettings('verbose', false,'solver','quadprog');
options.ipopt.max_iter = 10000;
simStates = [];
tSim = [];
uMpc = [];
for i = 1:M
    
    tspan = [tstart, tstart + Ts];
    cost = 0;
    constraints = [];
    urefk = uref(:, i:i+N);
    xrefk = xref(:, i:i+N);
    trefk = tref(:, i:i+N);
    for j = 1:N
        dx = sys.systemDynamics(trefk(:, j), xrefk(:, j), urefk(:, j));
        fdynx = sys.systemDynamics(0, x(:, j), u(:, j));
        [A, B] = sys.linearizeQuadrotor(xrefk(:, j), urefk(:, j));
        
        fhat =  dx + A*(x(:, j) - xrefk(:, j)) + B*(u(:, j) - urefk(:, j));
        
        cost = cost + (x(:, j) - xrefk(:, j))'*P*(x(:, j) - xrefk(:, j)) + (u(:, j) - urefk(:, j))'*R*(u(:, j) - urefk(:, j));
        constraints = [constraints, bounds.inputs.lb <= u(:, j) <= bounds.inputs.ub];
        constraints = [constraints, x(:, j+1) == x(:, j) + Ts*fhat];
        constraints = [constraints, x(:, 1) == xk];
    end
    cost = cost + (x(:, N+1) - xrefk(:, N+1))'*P*(x(:, N+1) - xrefk(:, N+1));
    %% optimization

    sol = optimize(constraints,cost,options);

    %% 
    if sol.problem == 0
        ctl.feas = true;
        ctl.xOpt = double(x);
        ctl.uOpt = double(u);
        ctl.JOpt = double(cost);
    else% sol.problem == 1
        ctl.feas = false;
        ctl.xOpt = [];
        ctl.uOpt = [];
        ctl.JOpt = [];
        warning('no feasible solution');
        keyboard;
    end
    uk =ctl.uOpt(:, 1);
    
    systemDynamics = @(t, x)sys.systemDynamics(t, x, uk);
    solSim = ode45(systemDynamics, tspan , xk);
    xk = solSim.y(:, end);
    tstart = solSim.x(end);
    
    simStates = [simStates, solSim.y];
    tSim = [tSim, solSim.x];
    uMpc = [uMpc, uk];
end

