function x = bSpline2(tGrid,xGrid,fGrid,t)
% x = bSpline2(tGrid,xGrid,fGrid,t)
%
% This function does piece-wise quadratic interpolation of a set of data.
% The quadratic interpolant is constructed such that the slope matches on
% both sides of each interval, and the function value matches on the lower
% side of the interval.
%
% INPUTS:
%   tGrid = [1, n] = time grid (knot points)
%   xGrid = [m, n] = function at each grid point in tGrid
%   fGrid = [m, n] = derivative at each grid point in tGrid
%   t = [1, k] = vector of query times (must be contained within tGrid)
%
% OUTPUTS:
%   x = [m, k] = function value at each query time
%
% NOTES:
%   If t is out of bounds, then all corresponding values for x are replaced
%   with NaN
%
[m,n] = size(xGrid);
k = length(t);
x = zeros(m, k);
% Figure out which segment each value of t should be on
[~, bin] = histc(t,[-inf,tGrid,inf]);
bin = bin - 1;
% Loop over each quadratic segment
for i=1:(n-1)
    idx = i==bin;
    if sum(idx) > 0
            h = (tGrid(i+1)-tGrid(i));
            xLow = xGrid(:,i);
            fLow = fGrid(:,i);
            fUpp = fGrid(:,i+1);
            delta = t(idx) - tGrid(i);
            x(:,idx) = bSpline2Core(h,delta,xLow,fLow,fUpp);
    end
end
% Replace any out-of-bounds queries with NaN
outOfBounds = bin==0 | bin==(n+1);
x(:,outOfBounds) = nan;
% Check for any points that are exactly on the upper grid point:
if sum(t==tGrid(end))>0
    x(:,t==tGrid(end)) = xGrid(:,end);
end
end
function x = bSpline2Core(h,delta,xLow,fLow,fUpp)
%
% This function computes the interpolant over a single interval
%
% INPUTS:
%   alpha = fraction of the way through the interval
%   xLow = function value at lower bound
%   fLow = derivative at lower bound
%   fUpp = derivative at upper bound
%
% OUTPUTS:
%   x = [m, p] = function at query times
%
%Fix dimensions for matrix operations...
col = ones(size(delta));
row = ones(size(xLow));
delta = row*delta;
xLow = xLow*col;
fLow = fLow*col;
fUpp = fUpp*col;
fDel = (0.5/h)*(fUpp-fLow);
x = delta.*(delta.*fDel + fLow) + xLow;
end