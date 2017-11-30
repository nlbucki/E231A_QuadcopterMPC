function traj = traj_gen_QR(obj)
% FUNCTION INPUTS:
x0 = [-10.5;-10.5;0;0;0;0];
xF = [0;0;0;0;0;0];
xU = inf(6,1);
xL = -xU;
uU = 15*ones(2,1);  % at least 10 to be able to compensate gravity
uL = zeros(2,1);
N = 80;

% Generate obstacle
% LATER: FUNCTION INPUT
O = Polyhedron('V',[-1 1; -20 1; -1 -2; -20 -2]);

%% Generate optimization problem

M = length(O);              % number of obstacles
numConstrObs = zeros(1,M);  % number of constraints to define each obstacle
for m=1:M
    numConstrObs(m) = size(O(m).H,1);
end
nx = length(xF);        % number of states
nu = length(uU);        % number of inputs

% Define optimization variables
x = sdpvar(nx, N+1);    % states
u = sdpvar(nu, N);      % control inputs
% Topt = sdpvar(1,N);     % optimal timestep variable over horizon
Topt = sdpvar(1,1);
lambda = sdpvar(sum(numConstrObs),N);   % dual variables corresponding to
                                        % the constraints defining the obs.
% A = sdpvar(nx,nx);
% B = sdpvar(nx,nu);
% [A,B]==obj.discrLinearizeQuadrotor(Topt,...
%     zeros(obj.nDof,1),obj.mQ*obj.g*ones(obj.nAct,1)./2)

% Specify cost function and constraints
q1 = 50;                % cost per time unit
q2 = 100;
R = eye(nu);            % cost for control inputs
dmin = 1;             % minimum safety distance
cost = 0;
constr = [x(:,1)==x0, x(:,N+1)==xF];
for k = 1:N
    cost = cost + u(:,k)'*R*u(:,k) ...
        + q1*Topt + q2*Topt^2;
%         + q1*Topt(k) + q2*Topt(k)^2;
    % Currently for the dynamics, an Euler discretization is used
    constr = [constr, xL<=x(:,k)<=xU, uL<=u(:,k)<=uU, ...
%         x(:,k+1)==x(:,k)+Topt(k)*systemDyn(obj,x(:,k),u(:,k)), ...
%         0.01<=Topt(k)<=0.375...
        x(:,k+1)==x(:,k)+Topt*systemDyn(obj,x(:,k),u(:,k)), ...
        0.01<=Topt<=0.375 ...
        ];
    % Include obstacle avoidance constraints
    for m=1:M
        lda = lambda(sum(numConstrObs(1:m-1))+1:sum(numConstrObs(1:m)),k);
        constr = [constr, ...
            (O.H(:,1:2)*x(1:2,k)-O.H(:,2+1))'*lda>=dmin, ...
            sum((O.H(:,1:2)'*lda).^2)<=1, ...
            lda>=zeros(size(lda))];
    end
end

% Specify solver and solve the optimization problem
options = sdpsettings('verbose', 1, 'solver', 'IPOPT');
opt = optimize(constr, cost, options);
% Assign output variables
traj.t = cumsum(value(Topt));
traj.u = value(u);
traj.x = value(x);
if opt.problem ~= 0
    % infeasible
    traj.t = [];
    traj.x = [];
    traj.u = [];
end

% Plot the resulting trajectory
figure
plot(traj.x(1,:),traj.x(2,:),'x-')
xlabel('y')
ylabel('z')
title('Generated path with obstacle avoidance')
hold on
plot(O,'alpha',0.5)

% Display the amount of time the planned motion would take
disp(['Reaching the target takes ' num2str(traj.t(end)) 's.'])
end

function ddx = systemDyn(obj,x,u)
    [fvec,gvec] = obj.quadVectorFields(x);
    ddx = fvec + gvec*u;
end