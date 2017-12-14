function [x, u] = interpolateTraj(sys, t, xdata, udata, tdata)


tLength = length(tdata);
N = tLength/2 - 1;
Tk = tdata(1:2:end);
hk = diff(Tk);
hk = hk(1);
Tm = tdata(2:2:end-1);
Xk = xdata(:, 1:2:end);
Xm = xdata(:, 2:2:end-1);

Uk = udata(:, 1:2:end);
Um = udata(:, 2:2:end-1);

tdiff = Tk-t;
indU = find(tdiff>=0);
indU = indU(1);
if t==0
    indL = indU;
else
    indL = indU-1;
end

tau = t - Tk(indL);

uk = Uk(:, indL);
xk = Xk(:, indL);
uk1 = Uk(:, indU);
xk1 = Xk(:, indU);
xkm = Xm(:, indL);
ukm = Um(:, indL);

fk = quadLoadDynamics(sys, xk, uk);
fk1 = quadLoadDynamics(sys, xk1, uk1);
fkm = quadLoadDynamics(sys, xkm, ukm);


u = (2/hk^2)*(tau - hk/2)*(tau - hk)*uk - (4/hk^2)*tau*(tau - hk)*Um(indL) + (2/hk^2)*tau*(tau - hk/2)*uk;
x = xk + fk*(tau/hk) + 0.5*(-3*fk + 4*fkm - fk1)*(tau/hk)^2 + (1/3)*(2*fk - 4*fkm + 2*fk1)*(tau/hk)^3;