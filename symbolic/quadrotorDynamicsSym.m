clear
clc

syms y z phi dy dz dphi F1 F2 real
syms mQ g JQ lQ real
syms y0 z0 phi0 dy0 dz0 dphi0 F10 F20 real % For linearization

params = [mQ;JQ;lQ;g];

X = [y;z;phi;dy;dz;dphi];
U = [F1;F2];
X0 = [y0;z0;phi0;dy0;dz0;dphi0];
U0 = [F10;F20];

fvec = [dy;dz;dphi;0;-g;0];

gvec = [zeros(3,2);-(1/mQ)*sin(phi), 0;(1/mQ)*cos(phi), 0;0, 1/JQ]*[1, 1;-lQ, lQ];

ddX = fvec + gvec*U;

Amat = jacobian(ddX,X); 
Amat = subs(Amat, [X',U'], [X0', U0']);
Bmat = jacobian(ddX, U);
Bmat = subs(Bmat, [X', U'], [X0', U0']);

matlabFunction(fvec, gvec, 'File', '../@Quadrotor/quadrotorVectorFields', 'Vars', {X, params});
matlabFunction(Amat, Bmat, 'File', '../@Quadrotor/quadrotorLinearDynamics', 'Vars', {X0, U0, params});




