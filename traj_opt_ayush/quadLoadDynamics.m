function  ddx = quadLoadDynamics(sys, x, u)

[fvec, gvec] = sys.quadVectorFields(x);
ddx = fvec + gvec*u;