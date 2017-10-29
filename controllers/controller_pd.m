function u = controller_pd(obj, t, x)
xd =zeros(6,1);

dof = obj.nDof;
nAct = obj.nAct;
lQ = obj.lQ;

kp_y = 0.1; kd_y = 0.2;
kp_z = 0.1; kd_z = 0.2;
kp_phi = 0.5; kd_phi = 0.8;

M = [1 1;
        -lQ lQ];
    
phi_c = -(kd_y*(xd(4)-x(4))+kp_y*(xd(1)-x(1)))/obj.g;

u2 = kp_phi*(phi_c-x(3)) + kd_phi*(0-x(6));
u1 = obj.mQ*(obj.g + kd_z*(xd(5)-x(5)) + kp_z*(xd(2)-x(2)));

u = inv(M)*[u1; u2];

end


