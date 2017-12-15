classdef Quadrotorload
% Dynamics for Quadrotor with load suspended through a cable
properties
    
    mQ@double
    JQ@double
    lQ@double
    hQ@double   % max. height QR
    wQ@double   % max. width QR
    
    g@double

    F1min@double
    F1max@double
    F2min@double
    F2max@double

    controller@function_handle
    controlParams@struct
    bounds@struct

    mL@double
    l@double
    wL@double   % max. width load (assumed to be height as well)
end

properties (Constant = true)
    nDof = 8;
    nAct = 2;
end
methods
	
	% class constructor
	function obj = Quadrotorload(params,varargin)
        
        if isfield(params, 'mQ')
            obj.mQ = params.mQ;
        else
            obj.mQ = 2;
        end
        
        if isfield(params, 'lQ')
            obj.lQ = params.lQ;
        else
            obj.lQ = 0.075;
        end
        
        if isfield(params, 'JQ')	
            obj.JQ = params.JQ;
        else
            obj.JQ = obj.mQ*obj.lQ^2;
        end
        
        if isfield(params, 'hQ')
			obj.hQ = params.hQ;
        else
            obj.hQ = 0.05;
        end
        
        if isfield(params, 'wQ')
			obj.wQ = params.wQ;
        else
            obj.wQ = 0.2;
        end
        
        if isfield(params, 'g')
            obj.g = params.g;
        else
            obj.g = 9.81;
        end

        if isfield(params, 'mL')
            obj.mL = params.mL;
        else
            obj.mL = 0.5;
        end
                
        if isfield(params, 'l')
            obj.l = params.l;
        else
            obj.l = 0.25;
        end
        
        if isfield(params, 'wL')
			obj.wL = params.wL;
        else
            obj.wL = 0.05;
        end
        
        if isfield(params, 'F1min')
			obj.F1min = params.F1min;
        else
            obj.F1min = 0;
        end
        
        if isfield(params, 'F1max')
			obj.F1max = params.F1max;
        else
            obj.F1max = 15;
        end
        
        if isfield(params, 'F2min')
			obj.F2min = params.F2min;
        else
            obj.F2min = 0;
        end
        
        if isfield(params, 'F2max')
			obj.F2max = params.F2max;
        else
            obj.F2max = 15;
        end
        
        if nargin > 1
            obj.bounds = varargin{1};
        else 
            % input bounds
            bounds.inputs.lb = [0; 0];
            bounds.inputs.ub = [obj.F1max;obj.F1max]; % --> *2bUpdated*
            
        	% state bounds 
            bounds.states.lb = [-100; -100; -pi; -pi; -100; -100; -100; -100];
            bounds.states.ub = [100; 100; pi; pi; 100; 100; 100; 100];
            
            obj.bounds = bounds;
        end
    end     
    
    % set property values
    function obj = setProperty(obj, propertyName, value)
        if isprop(obj, propertyName)
            set(obj, propertyName, value);
        else
            error([propertyName ' not a property of class ',class(obj)]);
        end
    end

    function sol = simulate(obj, tspan, x0, solver,varargin)
        if nargin > 4
            u = varargin{1};
            odefun = @(t,x)systemDynamics(obj, t, x,u);
        else
            odefun = @(t,x)systemDynamics(obj, t, x);
        end
        sol = solver(odefun, tspan, x0);
    end
    
    function [f, g] = quadVectorFields(obj, x)
        params = [obj.mQ;obj.JQ;obj.lQ;obj.mL;obj.l;obj.g];
        [f,g] = quadrotorloadVectorFields(x,params);
    end
    
    function [A, B] = linearizeQuadrotor(obj, x0, u0)
        params = [obj.mQ;obj.JQ;obj.lQ;obj.mL;obj.l;obj.g];
        [A, B] = quadrotorloadLinearDynamics(x0, u0, params);
    end
    
    function ddx = systemDynamics(obj, t, x, varargin)
        if nargin > 3
           u = varargin{1};                        
        else
           u = obj.controller(obj, t,x);
        end
        [fvec, gvec] = obj.quadVectorFields(x);
        ddx = fvec + gvec*u;
    end
    
    function u = calcControlInput(obj, t, x)
        u = obj.controller(obj, t,x);
    end
    
    [A, B] = discretizeLinearizeQuadrotorload(obj, Ts, xk, uk);
    [mpc_response] = mpc_load_Tracking(obj,x0,tref,xref,uref,varargin);
    [ctl] = solve_load_mpc(obj,Ts,xk,xrefk,urefk,FINAL_CONST_FLAG);
    [dlqr_response] = dlqr_load_Tracking(obj,x0,tref,xref,uref,type,varargin)
    
    % animation
    animateQuadrotorload(obj,opts_in)
    
    
end

end