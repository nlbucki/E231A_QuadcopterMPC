
if numStates == 6 % quadrotor
    state.lb = [-15;-15;deg2rad(-89);-5;-5;-10];
    state.ub = -state.lb;
    x0.lb = [0; -10; zeros(4, 1)];
    x0.ub = x0.lb;

    xf.lb = [10;10;0;0;0;0];
    xf.ub = xf.lb;
else % quadrotor with load
    state.lb = [-15;-15;deg2rad(-89);deg2rad(-89);-5;-5;-10;-10];
    state.ub = -state.lb;
    
    x0.lb = [-10; 5; zeros(6, 1)];
    x0.ub = x0.lb;

    xf.lb = [10;5;0;0;zeros(4, 1)];
    xf.ub = xf.lb;
end

input.ub = 15*ones(2,1);
input.lb = zeros(2,1);

% x0.lb = [-10.5;-10.5;0;0;0;0];


tf.ub = 20;
tf.lb = 2;

