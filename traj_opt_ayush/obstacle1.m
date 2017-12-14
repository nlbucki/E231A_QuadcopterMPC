function constraint = obstacle1(sys, constraint, x, x0, y0, r)

% circular obstacle

% x0 = 0; 
% y0 = 0;
% r = 2;
xQ = computeQuadPosition(sys, x);
constraint = [constraint, (x(1) - x0)^2 + (x(2) - y0)^2 >= r^2]; % constraint on the load
constraint = [constraint, (xQ(1) - x0)^2 + (xQ(2) - y0)^2 >= r^2];
