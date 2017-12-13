% Perform Hermite-Simpson Transcription
clear;close all;clc;
%% Add paths
user = 'ayush';
% addpath(genpath([pwd, '/@Quadrotorload']));
addpath(genpath([pwd, '/controllers/']));
addpath(genpath([pwd, '/gen/']));

params = struct;
sys = Quadrotorload(params);

%% 
numStates = sys.nDof;
numInputs = sys.nAct;
addBounds;
N = 20; % number of grid points 

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
end

% system dynamics (collocation constraints)
for i = 1:N
    constraints = [constraints, xMidPt(i) == 0.5*(xk(i) + xk1(i)) + (hk/8)*(fdyn(xk(i), uk(i)) - fdyn(xk1(i), uk1(i)))];
%     constraints = [constraints, uMidPt(i) == 0.5*(uk(i) + ul
    constraints = [constraints, xk1(i) == xk(i) + (1/6)*hk*(fdyn(xk(i), uk(i)) + 4*fdyn(xMidPt(i), uMidPt(i)) + fdyn(xk1(i), uk1(i)))];
end

constraints = [constraints, tf.lb<=Tf<=tf.ub]; % final time constr
constraints = [constraints, x0.lb <= x(:, 1) <= x0.ub]; % init condition
constraints = [constraints, xf.lb <= x(:, end) <= xf.ub]; % final condition


% constraints = [constraints, [0;-2.5;zeros(4, 1)]<=xk(ceil(N/2))<=[0;2.5;zeros(4, 1)]]; % window constraint
%% add cost
cost = 0;

for i = 1:N
    cost = cost + (hk/6)*(norm(uk(i))^2 + 4*norm(uMidPt(i))^2 + norm(uk1(i))^2);
end
%% call nlp solver
options = sdpsettings('verbose', true, 'solver', 'IPOPT');

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

%% run sim

