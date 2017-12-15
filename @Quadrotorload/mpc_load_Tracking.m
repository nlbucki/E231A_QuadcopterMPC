function [sys_response] = mpc_load_Tracking(obj,x0,tref,xref,uref,type,varargin)
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

    xrefk = xref(:,impc:(impc+obj.controlParams.mpc.N));
    urefk = uref(:,impc:(impc+obj.controlParams.mpc.N));
    
%     xrefk = [xref(:,impc), repmat(xref(:,impc+1),1,obj.controlParams.mpc.N)];
%     urefk = repmat(uref(:,impc),1,obj.controlParams.mpc.N+1);
%     if impc> obj.controlParams.mpc.M-obj.controlParams.mpc.N
%         FLAG = true;
%     else
%         FLAG = false;
%     end
    FLAG = false;
    Ts = Ts_ref(impc);
    ctlk = obj.solve_load_mpc(Ts,xk,xrefk,urefk,FLAG);
    uk = ctlk.uOpt(:,1);
    
    %% forward simulation
    % discrete -linear- simulation
    if strcmp(type,'DL')
        if flag_MODEL_ERRORS
            f0 = act_sys.systemDynamics([],xrefk(:,1),urefk(:,1));
            [A,B] = act_sys.linearizeQuadrotor(xrefk(:,1),urefk(:,1));
        else
            f0 = obj.systemDynamics([],xrefk(:,1),urefk(:,1));
            [A,B] = obj.linearizeQuadrotor(xrefk(:,1),urefk(:,1));
        end
        dxk = f0 + A*(xk-xrefk(:,1)) + B*(uk-urefk(:,1));
        xk_next = xk + Ts*dxk;
    end
    
	% discrete -nonlinear-simuation
    if strcmp(type,'DNL')
        if flag_MODEL_ERRORS
            f = act_sys.systemDynamics([],xk,uk);
        else
            f = obj.systemDynamics([],xk,uk);
        end
        xk_next = xk + Ts*f;
    end
    
    % continuous -nonlinear-simulation
    if strcmp(type,'CNL')
        if flag_MODEL_ERRORS
            sol = act_sys.simulate([tref(impc),tref(impc+1)], xk, solver,uk);
        else
            sol = obj.simulate([tref(impc),tref(impc+1)], xk, solver,uk);
        end
        xk_next = sol.y(:,end);
    end
    
    sys_response.x(:,impc+1) = xk_next;
    sys_response.u(:,impc) = uk;

end
    sys_response.t = tref;
end

