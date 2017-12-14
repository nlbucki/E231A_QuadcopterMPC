function xQ = computeQuadPosition(sys, x)

l = sys.l;
xL = x(1, :);
yL = x(2, :);
phi = x(3, :);

x = xL - l.*sin(phi);
y = yL + l.*cos(phi);

xQ = [x;y];