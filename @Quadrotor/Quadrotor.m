classdef Quadrotor

properties
    mQ@double
    JQ@double
    lQ@double
    g@double

    Fmin@double
    Fmax@double
    Mmin@double
    Mmax@double
    
    % bounds
    bounds@struct

    controller@function_handle
    controlParams@struct

end

properties (Constant = true)
    nDof = 6;
    nAct = 2;
end

methods
	% class constructor
	function obj = Quadrotor(params,varargin)
        
        if isfield(params, 'mQ')
            obj.mQ = params.mQ;
        else
            obj.mQ = 2;
        end
        
        if isfield(params, 'lQ')
            obj.lQ = params.lQ;
        else
            obj.lQ = 0.01;
        end
        
        if isfield(params, 'JQ')	
            obj.JQ = params.JQ;
        else
            obj.JQ = obj.mQ*obj.lQ^2;
        end
        
%         if isfield(params, 'lQ')
% 			obj.lQ = params.lQ;
%         end
        
        if isfield(params, 'g')
            obj.g = params.g;
        else
            obj.g = 9.81;
        end
        
        if nargin > 1
            obj.bounds = varargin{1};
        else 
            % input bounds
            bounds.inputs.lb = [0; 0];
            bounds.inputs.ub = [2*obj.mQ*obj.g; 2*obj.mQ*obj.g]; % --> *2bUpdated*
            
        	% state bounds 
            bounds.states.lb = [-100; -100; -pi/3; -100; -100; -100];
            bounds.states.ub = [100; 100; pi/3; 100; 100; 100];
            
            obj.bounds = bounds;
        end
                
    end     
%%    
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
        params = [obj.mQ;obj.JQ;obj.lQ;obj.g];
        [f,g] = quadrotorVectorFields(x,params);
    end
    
    function [A, B] = linearizeQuadrotor(obj, x0, u0)
%         params = [mQ;JQ;lQ;g];
        params = [obj.mQ;obj.JQ;obj.lQ;obj.g];
        [A, B] = quadrotorLinearDynamics(x0, u0, params);
    end
    
    function ddx = systemDynamics(obj, t, x, varargin)
        if nargin > 3
           u = varargin{1};
           [fvec, gvec] = obj.quadVectorFields(x);
           ddx = fvec + gvec*u;                        
        else
           u = obj.controller(obj, t,x);
           [fvec, gvec] = obj.quadVectorFields(x);
           ddx = fvec + gvec*u; 
        end
    end
    
    function u = calcControlInput(obj, t, x)
        u = obj.controller(obj, t,x);
    end

    function [Ad, Bd] = discrLinearizeQuadrotor(obj, Ts, xk, uk)
        [Ad, Bd] = discretizeLinearizeQuadrotor(obj, Ts, xk, uk);
    end

    % animation
    animateQuadrotor(obj,opts_in);
    
    
end

end