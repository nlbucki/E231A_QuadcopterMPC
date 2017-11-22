function [ref] = flat2state(obj,flats)
%% 
% Function to calculate the feedforward reference states and inputs for a
% given position trajectory using differential flatness

    ref.y   = flats.x(1);
    ref.dy  = flats.dx(1);
    ref.d2y = flats.d2x(1);
    ref.d3y = flats.d3x(1);
    ref.d4y = flats.d4x(1);
    ref.z   = flats.x(2);
    ref.dz  = flats.dx(2);
    ref.d2z = flats.d2x(2);
    ref.d3z = flats.d3x(2);
    ref.d4z = flats.d4x(2);
    
    ref.f   = obj.mQ*sqrt( ref.d2y^2 + (ref.d2z+obj.g)^2 );
    ref.phi = atan2(-ref.d2y,(ref.d2z+obj.g));
    
    num_dhpi = -(obj.g+ref.d2z)*ref.d3y + ref.d2y*ref.d3z;
    den_dphi = (ref.d2y)^2+(obj.g+ref.d2z)^2;
    ref.dphi = num_dhpi/den_dphi;

    num_d2phi = (-(-(obj.g+ref.d2z)*ref.d3y+ref.d2y*ref.d3z)*...
        (2*ref.d2y*ref.d3y + 2*(obj.g+ref.d2z)*ref.d3z) ...
        + (den_dphi)*(-(obj.g+ref.d2z)*ref.d4y+ref.d2y*ref.d4z));
    den_d2phi = den_dphi^2;
    ref.d2phi = num_d2phi/den_d2phi;

    ref.M = obj.JQ*ref.d2phi;
    F= [1 1; -obj.lQ obj.lQ]\[ref.f; ref.M];

    ref.F1 = F(1);
    ref.F2 = F(2);

end

