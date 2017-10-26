function [dx, varargout] = qrotor2D_dyn(t,x,data)

% Extracing parameters
% --------------------
% Dynamics of quadrotor 
m = data.params.mQ;
J = data.params.J;
g = data.params.g;
% e1 = data.params.e1;
% e2 = data.params.e2;
% e3 = data.params.e3;


% Extracing states
% ----------------
y = x(1);
z = x(2);
phi = x(3);
dy = x(4);
dz = x(5);
dphi = x(6);


f = [dy;
    dz;
    dphi;
    0;
    -g;
    0];
    
g = [0  0;
    0   0;
    0   0;
    -sin(phi)/m     0;
    cos(phi)/m      0;
    0       1/J];

u = qrotor2D_ctrl(t,x,xd,data);
    

% Computing dx
%-------------
dx = f + g*u;

if nargout <= 1
   fprintf('Sim time %0.4f seconds \n',t);
end
    
end












