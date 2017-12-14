function [ctl] = solve_mpc(obj,Ts,xk,xrefk,urefk)
%% function to Constrained Finite Time Optimal Control
yalmip('clear');

%% extracting params
Q = obj.controlParams.mpc.Q;
P = obj.controlParams.mpc.P;
R = obj.controlParams.mpc.R;
N = obj.controlParams.mpc.N;
% Ts = params.mpc.Ts;

%% creating optimization variables
% state variables
x = sdpvar(obj.nDof,N+1);
% input variables
u = sdpvar(obj.nAct,N);

% reference state variables
xref = sdpvar(obj.nDof,N+1);
% reference input variables
uref = sdpvar(obj.nAct,N+1);


%% cost function and constraints
% cost function
cost = 0;
% constraints
constraints = [];

% initial constraint
constraints = [constraints, x(:,1)==xk];
% looping over the N steps
for ii = 1:N
    f0 = obj.systemDynamics([],xrefk(:,ii),urefk(:,ii));
    [A,B] = obj.linearizeQuadrotor(xrefk(:,ii),urefk(:,ii));
    dxk = f0 + A*(x(:,ii)-xrefk(:,ii)) + B*(u(:,ii)-urefk(:,ii));
    
    constraints = [constraints,...
%             x(:,ii+1) == f0 + A*(x(:,ii)-xref(:,ii))+B*(u(:,ii)-uref(:,ii)),... % dynamics
            x(:,ii+1) == x(:,ii)+Ts*dxk,... % dynamics
            obj.bounds.inputs.lb <= u(:,ii), u(:,ii) <= obj.bounds.inputs.ub,... % input constraints
            obj.bounds.states.lb <= x(:,ii), x(:,ii) <= obj.bounds.states.ub... % state constraints
            ];
	% cost function
    cost = cost ...
        + (x(:,ii)-xrefk(:,ii))'*Q*(x(:,ii)-xrefk(:,ii))...
        + (u(:,ii)-urefk(:,ii))'*R*(u(:,ii)-urefk(:,ii));
end   
% terminal cost
cost = cost + (x(:,N+1)-xrefk(:,N+1))'*P*(x(:,N+1)-xrefk(:,N+1));

% reference constraints
% for ij = 1:N+1
%     constraints = [constraints,...
%                     xref(:,ij)==xrefk(:,ij),...
%                     uref(:,ij)==urefk(:,ij)];
% end

%% optimization
options = sdpsettings('verbose', true,'solver','quadprog');
options.ipopt.max_iter = 10000;
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
    warning('no feasible solutioin');
    keyboard;
end


end