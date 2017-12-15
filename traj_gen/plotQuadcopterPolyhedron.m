function plotQuadcopterPolyhedron(x,QR,L,obj,O,color)
% x states, QR Quadrotor Polyhedron, L Load Polyhedron, obj: Quadrotor
% object, O: Obstacle
% 
    [~,N]=size(x);
    N=N-1;
    [~,M]=size(O);
    plot(x(1,:),x(2,:),'x-')
    xlabel('y')
    ylabel('z')
    title('Generated path with obstacle avoidance')
    hold on
    for m=1:M
        plot(O{m},'alpha',0.5)
    end
    Rot = @(angle) [cos(angle) -sin(angle); sin(angle) cos(angle)];
    for k=1:N+1
        plot(Polyhedron('H',[QR.H(:,1:2)*Rot(x(4,k))',...
            QR.H(:,2+1)+QR.H(:,1:2)*Rot(x(4,k))'*...
            (x(1:2,k)+Rot(x(3,k))*[0; obj.l])]),...
            'color',color,'alpha',0)
        plot(Polyhedron('H',[L.H(:,1:2)*Rot(x(3,k))',...
            L.H(:,2+1)+L.H(:,1:2)*Rot(x(3,k))'*x(1:2,k)]),...
            'color',color,'alpha',0)
    end
    axis equal