function [ctl] = solve_cftoc(xk,xrefk,urefk,sys,params)
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
uref = sdpvar(sys.nAct,N+1);


%% cost function and constraints
% cost function
cost = 0;
% constraints
constraints = [];

% initial constraint
constraints = [constraints, x(:,1)==xk];
% looping over the N steps
for ii = 1:N
    f0 = sys.systemDynamics([],xrefk(:,ii),urefk(:,ii));
    [A,B] = sys.discretizeLinearizeQuadrotor(Ts, xrefk(:,ii),urefk(:,ii));
    constraints = [constraints,...
            x(:,ii+1) == f0 + A*(x(:,ii)-xref(:,ii))+B*(u(:,ii)-uref(:,ii)),... % dynamics
            sys.bounds.inputs.lb <= u(:,ii), u(:,ii) <= sys.bounds.inputs.ub,... % input constraints
            sys.bounds.states.lb <= x(:,ii), x(:,ii) <= sys.bounds.states.ub... % state constraints
            ];
	% cost function
    cost = cost ...
        + (x(:,ii)-xref(:,ii))'*Q*(x(:,ii)-xref(:,ii))...
        + (u(:,ii)-uref(:,ii))'*R*(u(:,ii)-uref(:,ii));
end   
% terminal cost
cost = cost + (x(:,N+1)-xref(:,N+1))'*P*(x(:,N+1)-xref(:,N+1));

% reference constraints
for ij = 1:N+1
    constraints = [constraints,...
                    xref(:,ij)==xrefk(:,ij),...
                    uref(:,ij)==urefk(:,ij)];
end

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