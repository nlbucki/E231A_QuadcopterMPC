function u = controller(obj, t, x)

dof = obj.nDof;
nAct = obj.nAct;

% params = obj.controlParams;

[A,B] = obj.discretizeLinearizeQuadrotor(0.1, zeros(dof, 1), ...
    obj.mQ*obj.g*ones(nAct, 1)./2);

K = dlqr(A,B, 5*eye(dof), eye(nAct));

u = -K*x + [obj.mQ*obj.g;obj.mQ*obj.g]./2;
end