function [Ad, Bd] = discretizeLinearizeQuadrotor(obj, Ts, xk, uk)
% This function discretizes and lienarizes the quadrotor system dynamics to
% the form
% x(k+1) = Ad*x(k) + Bd*u(k).

[A, B] = obj.linearizeQuadrotor(xk, uk);
% Ad = eye(obj.nDof) + Ts*A;
% Bd = Ts*B;
% Discretize the matrices
if rank(A) - min(size(A)) == 0
    % Standard equations (Bd not via integral but special equation for
    % non-singular A matrix)
    Ad = expm(A*Ts);
    Bd = A\(Ad-eye(size(Ad,1)))*B;
else
    % Computation via one big exponential matrix
    exponMat = expm([A, B; zeros(size(B,2),size(A,2)+size(B,2))]*Ts);
    Ad = exponMat(1:size(A,1),1:size(A,2));
    Bd = exponMat(1:size(B,1),(size(A,2)+1):(size(A,2)+size(B,2)));
end
end
