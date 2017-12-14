function x = pwPoly3(tGrid,xGrid,fGrid,t)
% x = pwPoly3(tGrid,xGrid,fGrid,t)
%
% This function does piece-wise quadratic interpolation of a set of data,
% given the function value at the edges and midpoint of the interval of
% interest.
%
% INPUTS:
%   tGrid = [1, 2*n-1] = time grid, knot idx = 1:2:end
%   xGrid = [m, 2*n-1] = function at each grid point in time
%   fGrid = [m, 2*n-1] = derivative at each grid point in time
%   t = [1, k] = vector of query times (must be contained within tGrid)
%
% OUTPUTS:
%   x = [m, k] = function value at each query time
%
% NOTES:
%   If t is out of bounds, then all corresponding values for x are replaced
%   with NaN
%

nGrid = length(tGrid);
if mod(nGrid-1,2)~=0 || nGrid < 3
    error('The number of grid-points must be odd and at least 3');
end

% Figure out sizes
n = floor((length(tGrid)-1)/2);
m = size(xGrid,1);
k = length(t);
x = zeros(m, k);

% Figure out which segment each value of t should be on
edges = [-inf, tGrid(1:2:end), inf];
[~, bin] = histc(t,edges);

% Loop over each quadratic segment
for i=1:n
    idx = bin==(i+1);
    if sum(idx) > 0
        kLow = 2*(i-1) + 1;
        kMid = kLow + 1;
        kUpp = kLow + 2;
        h = tGrid(kUpp)-tGrid(kLow);
        xLow = xGrid(:,kLow);
        fLow = fGrid(:,kLow);
        fMid = fGrid(:,kMid);
        fUpp = fGrid(:,kUpp);
        alpha = t(idx) - tGrid(kLow);
        x(:,idx) = cubicInterp(h,xLow, fLow, fMid, fUpp,alpha);
    end
end

% Replace any out-of-bounds queries with NaN
outOfBounds = bin==1 | bin==(n+2);
x(:,outOfBounds) = nan;

% Check for any points that are exactly on the upper grid point:
if sum(t==tGrid(end))>0
    x(:,t==tGrid(end)) = xGrid(:,end);
end

end


function x = cubicInterp(h,xLow, fLow, fMid, fUpp,del)
%
% This function computes the interpolant over a single interval
%
% INPUTS:
%   h = time step (tUpp-tLow)
%   xLow = function value at tLow
%   fLow = derivative at tLow
%   fMid = derivative at tMid
%   fUpp = derivative at tUpp
%   del = query points on domain [0, h]
%
% OUTPUTS:
%   x = [m, p] = function at query times
%

%%% Fix matrix dimensions for vectorized calculations
nx = length(xLow);
nt = length(del);
xLow = xLow*ones(1,nt);
fLow = fLow*ones(1,nt);
fMid = fMid*ones(1,nt);
fUpp = fUpp*ones(1,nt);
del = ones(nx,1)*del;

a = (2.*(fLow - 2.*fMid + fUpp))./(3.*h.^2);
b = -(3.*fLow - 4.*fMid + fUpp)./(2.*h);
c = fLow;
d = xLow;

x = d + del.*(c + del.*(b + del.*a));

end
