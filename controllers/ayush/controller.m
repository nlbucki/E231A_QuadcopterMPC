function u = controller(obj, t, x)

dof = obj.nDof;
nAct = obj.nAct;

params = obj.controlParams;

uff = params.ufcn(t);
xref = params.xfcn(t);
[A,B] = obj.linearizeQuadrotor(zeros(dof, 1), obj.mQ*obj.g*ones(nAct, 1)./2);

K = lqr(A,B, 1*eye(dof), 1*eye(nAct));

u = -K*(x-xref) + uff;%[obj.mQ*obj.g;obj.mQ*obj.g]./2;
end
