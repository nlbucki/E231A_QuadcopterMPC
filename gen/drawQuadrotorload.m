function drawQuadrotorload(x,l)
    yL = x(1);
    zL = x(2);
    phiL = x(3);
    phiQ = x(4);
    
    yQ = yL-l*sin(phiL);
    zQ = zL+l*cos(phiL);
    
    s.L = 0.175;
    s.R = 0.05;
    
    base = [[s.L; 0], [-s.L; 0], [s.L; s.R], [-s.L; s.R]];
    R = [cos(phiQ), -sin(phiQ);
            sin(phiQ), cos(phiQ)];
    
    points = [yQ;zQ] + R*base;  
    
    cable = [ [yQ; zQ], [yL; zL]];
    
    plot([points(1,1), points(1,2)], [points(2,1), points(2,2)],'Color','r', 'LineWidth',2); hold on ;
    plot([points(1,1), points(1,3)], [points(2,1), points(2,3)],'Color','b', 'LineWidth',2);
    plot([points(1,2), points(1,4)], [points(2,2), points(2,4)],'Color','b', 'LineWidth',2);
    plot(cable(1,:),cable(2,:),'Color','b');
    scatter(yL,zL,40,'ks','filled');
    
    
    grid on;
    xlabel('Y');
    ylabel('Z');

end