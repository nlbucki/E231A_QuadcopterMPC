function [ctl] = solve_for_cftoc(xk,xrefk,urefk,sys,params)
%% function to Constrained Finite Time Optimal Control
yalmip('clear');

%% extracting params
Q = params.mpc.Q;
P = params.mpc.P;
R = params.mpc.R;
N = params.mpc.N;
Ts = params.mpc.Ts;

%% creating optimization variables
% state variables
x = sdpvar(sys.nDof,N+1);
% input variables
u = sdpvar(sys.nAct,N);

% reference state variables
xref = sdpvar(sys.nDof,N+1);
% reference input variables
uref = sdpvar(sys.nAct,N);


%% cost function and constraints
% cost function
cost = 0;
% constraints
constraints = [];

% initial constraint
constraints = [constraints, x(:,1)==xk];
% looping over the N steps
for ii = 1:N
    f0 = sys.systemDynamics([],xrefk,urefk);
    [A,B] = sys.discretizeLinearizeQuadrotor(Ts, xrefk,urefk);
    constraints = [constraints,...
            x(:,ii+1) == f0 + A*(x(:,ii)-xrefk)+B*(u(:,ii)-urefk),... % dynamics
            sys.Fmin*ones(sys.nAct,1) <= u(:,ii), u(:,ii) <= sys.Fmax*ones(sys.nAct,1),... % input constraints
            ];
	% cost function
    cost = cost ...
        + (x(:,ii)-xrefk)'*Q*(x(:,ii)-xrefk)...
        + (u(:,ii)-urefk)'*R*(u(:,ii)-urefk);
end   
% terminal cost
cost = cost + (x(:,N+1)-xrefk)'*P*(x(:,N+1)-xrefk);

%% solving 
options = sdpsettings('verbose', false, 'solver', 'quadprog');
sol = optimize(constraints,cost,options);

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
    warning('no feasible solutioin');
    keyboard;
end


end