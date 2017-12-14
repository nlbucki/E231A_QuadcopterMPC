function constraint = superEllipse(sys, constraint, x, x0, y0, a, b, d)

% SUPERELLIPSE Adds a super ellipse obstacle constraint
% INPUTS: 
% x - Quadrotor state
% (x0, y0) - Center of super ellipse
% (a, b) - Semi major/minor axes of super ellipse
% d - degree of super ellipse (even)
if nargin == 7
    d = 4;
end
if mod(d, 2)
    error('degree of super ellipse should be even')
end
% super ellipse obstacle

xQ = computeQuadPosition(sys, x);
x1 = xQ(1, :) + sys.lQ.*cos(x(4, :));
y1 = xQ(2, :) + sys.lQ.*sin(x(4, :));
x2 = xQ(1, :) - sys.lQ.*cos(x(4, :));
y2 = xQ(2, :) - sys.lQ.*sin(x(4, :));

constraint = [constraint, ((x(1, :) - x0)./a).^d + ((x(2, :) - y0)./b).^d >= 1]; % constraint on the load
constraint = [constraint, ((xQ(1, :) - x0)./a).^d + ((xQ(2, :) - y0)./b).^d >= 1];

constraint = [constraint, ((x1(1, :) - x0)./a).^d + ((y1(1, :) - y0)./b).^d >= 1]; 
constraint = [constraint, ((x2(1, :) - x0)./a).^d + ((y2(1, :) - y0)./b).^d >= 1]; 
