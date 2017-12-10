classdef Quadrotorload
% Dynamics for Quadrotor with load suspended through a cable
properties
    
    mQ@double
    JQ@double
    lQ@double
    hQ@double   % max. height QR
    wQ@double   % max. width QR
    
    g@double

    Fmin@double
    Fmax@double
    Mmin@double
    Mmax@double

    controller@function_handle
    controlParams@struct

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
	function obj = Quadrotorload(params)
        
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
        
        if isfield(params, 'Fmin')
			obj.Fmin = params.Fmin;
        else
            obj.Fmin = 0;
        end
        
        if isfield(params, 'Fmax')
			obj.Fmax = params.Fmax;
        else
            obj.Fmax = obj.g*(obj.mQ+obj.mL)*10;
        end
        
        if isfield(params, 'Mmin')
			obj.Mmin = params.Mmin;
        else
            obj.Mmin = -obj.lQ*obj.Fmax;
        end
        
        if isfield(params, 'Mmax')
			obj.Mmax = params.Mmax;
        else
            obj.Mmax = obj.lQ*obj.Fmax;
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

    function sol = simulate(obj, tspan, x0, solver)
        odefun = @(t,x)systemDynamics(obj, t, x);
        sol = solver(odefun, tspan, x0);
    end
    
    function [f, g] = quadVectorFields(obj, x)
%         params = [mQ;JQ;lQ;g];
        params = [obj.mQ;obj.JQ;obj.lQ;obj.mL;obj.l;obj.g];
        [f,g] = quadrotorloadVectorFields(x,params);
    end
    
    function [A, B] = linearizeQuadrotor(obj, x0, u0)
%         params = [mQ;JQ;lQ;g];
        params = [obj.mQ;obj.JQ;obj.lQ;obj.mL;obj.l;obj.g];
        [A, B] = quadrotorloadLinearDynamics(x0, u0, params);
    end
    
    function ddx = systemDynamics(obj, t, x)
        
        u = obj.controller(obj, t,x);
        [fvec, gvec] = obj.quadVectorFields(x);
        ddx = fvec + gvec*u;
        
    end
    
    function u = calcControlInput(obj, t, x)
        u = obj.controller(obj, t,x);
    end
    
    [A, B] = discretizeLinearizeQuadrotorload(obj, Ts, xk, uk);
    % TODO:
%     obj = setLoadMass(mL); 

    % animation
    animateQuadrotorload(obj,t,x,varargin)
    
    
end

end