% Perform Hermite-Simpson Transcription
clear;close all;clc;
%% Add paths
user = 'ayush';
% addpath(genpath([pwd, '/@Quadrotorload']));
addpath(genpath([pwd, '/controllers/']));
addpath(genpath([pwd, '/gen/']));
soln = load([pwd, '/traj_opt_ayush/soln.mat']);
params = struct;
% params.mQ = 3;
% params.mL = 1;
% params.lQ = 0.1;
sys = Quadrotorload(params);

%% 
numStates = sys.nDof;
numInputs = sys.nAct;
addBounds;
N = 30; % number of grid points 

x = sdpvar(numStates, 2*N+1);
u = sdpvar(numInputs, 2*N+1);
Tf = sdpvar(1, 1);

xMidPt = @(ind)x(:, 2*ind); % every even index is a mid point
uMidPt = @(ind)u(:, 2*ind);
xk = @(ind)x(:, 2*ind-1); % every odd index is a grid point
uk = @(ind)u(:, 2*ind-1);
xk1 = @(ind)x(:, 2*ind+1);
uk1 = @(ind)u(:, 2*ind+1);

% fdyn = @(x, u)sys.systemDynamics(0, x, u); % dynamics are independent of time
fdyn = @(x, u)quadLoadDynamics(sys, x, u);
hk = Tf/N; % uniform spacing

%% add constraints:
constraints = [];
% state constraints
for i = 1:2*N+1
    constraints = [constraints, state.lb<=x(:, i)<=state.ub];
    constraints = [constraints, input.lb<=u(:, i)<=input.ub];
%     constraints = obstacle1(sys,constraints, x(:, i), 0, 0, 2);
%     constraints = obstacle1(sys, constraints, x(:, i), 0, 3, 2);
    constraints = superEllipse(sys, constraints, x(:, i), 0, -15, 3, 14.9);
    constraints = superEllipse(sys, constraints, x(:, i), 0, 15, 3, 14.9);
end

% system dynamics (collocation constraints)
for i = 1:N
    constraints = [constraints, xMidPt(i) == 0.5*(xk(i) + xk1(i)) + (hk/8)*(fdyn(xk(i), uk(i)) - fdyn(xk1(i), uk1(i)))];
    constraints = [constraints, xk1(i) == xk(i) + (1/6)*hk*(fdyn(xk(i), uk(i)) + 4*fdyn(xMidPt(i), uMidPt(i)) + fdyn(xk1(i), uk1(i)))];
end

constraints = [constraints, tf.lb<=Tf<=tf.ub]; % final time constr
constraints = [constraints, x0.lb <= x(:, 1) <= x0.ub]; % init condition
constraints = [constraints, xf.lb <= x(:, end) <= xf.ub]; % final condition


% constraints = [constraints, [0;-2.5;zeros(6, 1)]<=xk(ceil(N/2))<=[0;2.5;zeros(6, 1)]]; % window constraint
%% add cost
cost = 0;

for i = 1:N
    cost = cost + (hk/6)*(norm(uk(i))^2 + 4*norm(uMidPt(i))^2 + norm(uk1(i))^2);
end
cost = cost + Tf^2;
%% call nlp solver
xL = linspace(x0.lb(1), xf.lb(1), 2*N+1);
xinit = [xL;zeros(numStates-1, 2*N+1)];
uinit = (sys.mQ + sys.mL)*sys.g*ones(numInputs, 2*N+1);
Tfinit = 10;
options = sdpsettings('solver', 'IPOPT', 'usex0', true, 'showprogress', 1,'verbose', 3);
assign(x,soln.x);
assign(u, soln.u);
assign(Tf, soln.Tf);
% assign(x, xinit);
% assign(u, uinit);
% assign(Tf, Tfinit);

options.ipopt.mu_strategy      = 'adaptive';
options.ipopt.max_iter         = 1000;
options.ipopt.tol              = 1e-3;
options.ipopt.linear_solver    = 'ma57';
options.ipopt.ma57_automatic_scaling = 'yes';
options.ipopt.linear_scaling_on_demand = 'no';
options.ipopt.hessian_approximation = 'limited-memory';
options.ipopt.limited_memory_update_type = 'bfgs';  % {bfgs}, sr1
options.ipopt.limited_memory_max_history = 10;  % {6}

tic
opt = optimize(constraints, cost, options);
toc

%% extract solution
x = double(x);
u = double(u);
hk = double(hk);
Tf = double(Tf);
tspan = linspace(0, Tf, 2*N+1);
ddxk = zeros(numStates, 2*N+1);
for i = 1:2*N+1
    ddxk(:, i) = quadLoadDynamics(sys, x(:, i), u(:, i));
end

%% spline interp
Ninterp = 100;
tinterp = linspace(0, Tf, Ninterp);
xinterp = zeros(numStates, Ninterp);
uinterp = zeros(numInputs, Ninterp);
for i = 1:Ninterp
    xinterp(:, i) = pwPoly3(tspan, x, ddxk, tinterp(i)); % From: https://github.com/MatthewPeterKelly/OptimTraj
end

for i = 1:Ninterp
    uinterp(:, i) = pwPoly2(tspan, u, tinterp(i)); % From: https://github.com/MatthewPeterKelly/OptimTraj
end
% 
xinterpfcn = @(t)pwPoly3(tspan, x, ddxk, t);
uinterpfcn = @(t)pwPoly2(tspan, u, t);
%% run sim

controlParams.ufcn = uinterpfcn;
controlParams.xfcn = xinterpfcn;
sys.controller = @controller;
sys.controlParams = controlParams;


tspanSim = [0,tspan(end)];
x0Sim = x(:, 1);
solver = @ode45;
sol = sys.simulate(tspanSim, x0Sim, solver);

data.x = sol.y;
data.t = sol.x;
xinterpSim = zeros(numStates, length(sol.x));
for i = 1:length(sol.x)
    xinterpSim(:, i) = xinterpfcn(sol.x(i));
end

data.xd = xinterpSim';
data.td = sol.x';

animateQuadrotorload(sys, data);