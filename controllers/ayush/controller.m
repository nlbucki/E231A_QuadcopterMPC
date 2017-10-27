function u = controller(obj, t, x)

dof = 6;
nAct = 2;

% params = obj.controlParams;

[A,B] = obj.linearizeQuadrotor(zeros(dof, 1), obj.mQ*obj.g*ones(nAct, 1)./2);

K = lqr(A,B, 5*eye(dof), eye(nAct));

u = -K*x + [obj.mQ*obj.g;obj.mQ*obj.g]/2;