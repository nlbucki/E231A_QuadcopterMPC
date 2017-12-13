function u = controller2(obj, t, x)

dof = obj.nDof;
nAct = obj.nAct;

% params = obj.controlParams;
[xref,uref]  = generate_ref_trajectory(t, obj);
[A,B] = obj.linearizeQuadrotor(xref, uref);

if  sum(isnan(A(:)))
    keyboard
end
K = lqr(A,B, 100*eye(dof), 0.1*eye(nAct));

% u = -K*x + uref;
u = -K*(x-xref) +uref;

end
