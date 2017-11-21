function [ctl] = solve_for_cftoc(xk,xrefk,urefk,sys,params)
%% function to Constrained Finite Time Optimal Control
yalmip('clear');

%% creating optimization variables
% state variables
x = sdpvar(sys.nDof,params.mpc.N+1);
% input variables
u = sdpvar(sys.nAct,params.mpc.N);

% reference state variables
xref = sdpvar(sys.nDof,params.mpc.N+1);
% reference input variables
uref = sdpvar(sys.nAct,params.mpc.N);


%% cost function and constraints
% cost function
cost = 0;
% constraints
constraints = [];

% initial constraint
constraints = [constraints, x(:,1)==xk];
% looping over the N steps
for ii = 1:params.mpc.N
    [A,B] = sys.discretizeLinearizeQuadrotor(params.mpc.Ts, xrefk,urefk);
    constraints = [constraints,...
            % dynamics
            x(:,ii+1) == A*x(:,ii)+B*u(:,ii),...
            % input constraints
            sys.Fmin*ones(sys.nAct,1) <= u(:,ii), u(:,ii) <= sys.Fmax*ones(sys.nAct,1),...
            ];
	% cost function
    cost = cost ...
        + (x(:,ii)-xrefk)'*params.mpc.Q*(x(:,ii)-xrefk)...
        + (u(:,ii)-urefk)'*params.mpc.R*(u(:,ii)-urefk);
end   
% terminal cost
cost = cost + (x(:,params.mpc.N+1)-xrefk)'*params.mpc.P*(x(:,params.mpc.N+1)-xrefk);

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