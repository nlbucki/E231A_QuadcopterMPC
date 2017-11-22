function[xref,uref] = generate_ref_trajectory(sys,params)
%% 
% function to generate reference trajectory

%%
time = 0:params.Ts:(params.Tf+params.N*params.Ts);

traj = @(t) sin_traj(t);
% traj = @(t) circ_traj(t);

xref = [];
uref = [];
for it = time
    [ref] = sys.flat2state(traj(it));
    xref_ = [ref.y; ref.z; ref.phi; ref.dy; ref.dz; ref.dphi];
    uref_ = [ref.F1; ref.F2];
    
    xref = [xref, xref_];
    uref = [uref, uref_];
end


end

function [flats] = sin_traj(t)
y0 = 0;
z0 = 0;
vy0 = 1;
ay0 = 0.01;
zr = 0.5;

f = 0.5;
w = 2*pi*f;

flats.x = [y0+vy0*t+ay0*t^2;z0+zr*sin(w*t)]; 
flats.dx = [vy0+2*ay0*t;zr*1E0.*w.*cos(t.*w)];
flats.d2x = [0+2*ay0;(-zr*1E0).*w.^2.*sin(t.*w)];
flats.d3x = [0;(-zr*1E0).*w.^3.*cos(t.*w)];
flats.d4x = [0;zr*1E0.*w.^4.*sin(t.*w)];
end

function [flats] = circ_traj(t)
y0 = 0;
z0 = 0;

ry = 0.5;
rz = 0.5;

f = 0.5;
w = 2*pi*f;

flats.x   = [y0+ry.*sin(t.*w),z0+rz.*cos(t.*w)];
flats.dx  = [ry.*w.*cos(t.*w),(-1).*rz.*w.*sin(t.*w)];
flats.d2x = [(-1).*ry.*w.^2.*sin(t.*w),(-1).*rz.*w.^2.*cos(t.*w)];
flats.d3x = [(-1).*ry.*w.^3.*cos(t.*w),rz.*w.^3.*sin(t.*w)];
flats.d4x = [ry.*w.^4.*sin(t.*w),rz.*w.^4.*cos(t.*w)];

% flats.x = flip(flats.x);
% flats.dx = flip(flats.dx);
% flats.d2x = flip(flats.d2x);
% flats.d3x = flip(flats.d3x);
% flats.d4x = flip(flats.d4x);
end

