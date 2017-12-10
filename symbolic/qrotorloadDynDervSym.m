syms phiL dphiL real

p = [sin(phiL); -cos(phiL)]
dp = jacobian(p,phiL)*dphiL

dp_dot_dp = simplify(dot(dp,dp));

syms phiQ dphiQ f mQ L real
e3 = [0;1]

R = [cos(phiQ) -sin(phiQ); 
    sin(phiQ) cos(phiQ)];

F = dot(p,f*R*e3);

rhs = simplify((F-mQ*L*dp_dot_dp)*p)



