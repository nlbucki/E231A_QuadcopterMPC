function [Ad, Bd] = discretizeLinearizeQuadrotor(obj, Ts, xk, uk)


[A, B] = obj.linearizeQuadrotor(xk, uk);
Ad = eye(obj.nDof) + Ts*A;
Bd = Ts*B;

end