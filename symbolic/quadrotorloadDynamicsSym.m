clear
clc

syms yL zL phiQ phiL dyL dzL dphiQ dphiL f M F1 F2 real
syms mQ g JQ lQ mL l real
syms yL0 zL0 phiQ0 phiL0 dyL0 dzL0 dphiQ0 dphiL0 F10 F20 real % For linearization

params = [mQ;JQ;lQ;mL;l;g];

X = [yL;zL;phiL;phiQ;dyL;dzL;dphiL;dphiQ];
U = [F1;F2];
X0 = [yL0;zL0;phiL0;phiQ0;dyL0;dzL0;dphiL0;dphiQ0];
U0 = [F10;F20];

fvec = [dyL;
        dzL;
        dphiL;
        dphiQ;
        (-mQ/(mQ+mL))*l*dphiL^2*sin(phiL);
        (mQ/(mQ+mL))*l*dphiL^2*cos(phiL)-g;
        0;
        0];

gvec = [zeros(4,2);
    -(sin(phiL)/(mQ+mL))*cos(phiQ-phiL), 0;
    (cos(phiL)/(mQ+mL))*cos(phiQ-phiL), 0;
    sin(phiQ-phiL)/(mQ*l),  0;
    0, 1/JQ]*[1, 1;-lQ, lQ];

ddX = fvec + gvec*U;

Amat = jacobian(ddX,X); 
Amat = subs(Amat, [X',U'], [X0', U0']);
Bmat = jacobian(ddX, U);
Bmat = subs(Bmat, [X', U'], [X0', U0']);

matlabFunction(fvec, gvec, 'File', '../@Quadrotorload/quadrotorloadVectorFields', 'Vars', {X, params});
matlabFunction(Amat, Bmat, 'File', '../@Quadrotorload/quadrotorloadLinearDynamics', 'Vars', {X0, U0, params});




