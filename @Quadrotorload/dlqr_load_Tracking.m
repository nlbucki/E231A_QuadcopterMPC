function [sys_response] = dlqr_load_Tracking(obj,x0,tref,xref,uref,type,varargin)
%% 
% Code to implement MPC for trajectory tracking of quadrotor position

%% inputs
flag_MODEL_ERRORS= false;

if nargin > 6
    flag_MODEL_ERRORS = true;
    act_sys = varargin{1};
else

%% Control 
% system response
sys_response.x = zeros(obj.nDof,obj.controlParams.mpc.M+1);
sys_response.u = zeros(obj.nAct,obj.controlParams.mpc.M);
sys_response.x(:,1) = x0;
solver = @ode45;

Ts_ref = diff(tref);

% calculating input over the loop
for impc = 1:obj.controlParams.mpc.M
    fprintf('calculting input for T = %.4f\n',tref(impc+1));
    
    %% optimizing for input
    xk = sys_response.x(:,impc);

%     xrefk = xref(:,impc:(impc+obj.controlParams.mpc.N));
%     urefk = uref(:,impc:(impc+obj.controlParams.mpc.N));
    
    xrefk = xref(:,impc);
    urefk = uref(:,impc);

    Ts = Ts_ref(impc);    
    % Linearize about desired trajectory position
    [A,B] = obj.discretizeLinearizeQuadrotorload(Ts, xrefk,urefk);

    K = dlqr(A,B, 5*eye(obj.nDof), eye(obj.nAct));

    uk = -K*(xk-xrefk) + urefk;

    %% forward simulation
    if flag_MODEL_ERRORS
        sol = act_sys.simulate([tref(impc),tref(impc+1)], xk, solver,uk);
    else
        sol = obj.simulate([tref(impc),tref(impc+1)], xk, solver,uk);
    end
    xk_next = sol.y(:,end);

    
    sys_response.x(:,impc+1) = xk_next;
    sys_response.u(:,impc) = uk;

end
    sys_response.t = tref;
end

