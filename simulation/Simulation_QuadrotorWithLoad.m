%% Simulation of Quadcopter
clear;
clc;
addpath(genpath('C:\Users\mx190\Documents\GitHub\E231A_QuadcopterMPC'))
%% Create Object
params = struct;
sys = Quadrotorload(params);
sys.controller = @controller_dlqr_path;
%% Trajectory Generating
%traj = traj_gen_QRL_polyhedron(sys);
load('traj_gen_QRL.mat');
Topt=traj.t(2)-traj.t(1);
%% Simulation
time = traj.t;
states = traj.x;
control = traj.u;
sys.controlParams = struct('time',time,'states',states,'control',control);
x0=sys.controlParams.states(:,1);
N=length(sys.controlParams.states(1,:))-1;
x=NaN(8,N+1);
u=NaN(2,N);
x(:,1)=x0;
for k=1:N
    t=traj.t(k);
    u(:,k)= controller_dlqr_path(sys, t, x(:,k));
    x(:,k+1)=x(:,k)+Topt*systemDyn(sys,x(:,k),u(:,k));
end

%% Plot
figure
plot(x(1,:),x(2,:),'x-')
xlabel('y')
ylabel('z')
title('Generated path with obstacle avoidance')
hold on
O ={Polyhedron('V',[-2 1; -20 1; -2 -2; -20 -2])};
plot(O{1},'alpha',0.5)


function ddx = systemDyn(obj,x,u)
[fvec,gvec] = obj.quadVectorFields(x);
ddx = fvec + gvec*u;
end