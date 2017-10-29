function animateQuadrotor(obj,torig,x,varargin)
    RATE = 25 * 2;
%==========================================
% initialize the animation figure and axes
%==========================================
    figure_x_limits = [-3 3];
    figure_y_limits = [-3 3];
    figure_z_limits = [0 1] ;
    fig1 = figure;

    set(0,'Units','pixels')
    scnsize = get(0,'ScreenSize');

    screen_width = scnsize(3);
    screen_height = scnsize(4);

    % find the minimum scaling factor
    figure_x_size = figure_x_limits(2) - figure_x_limits(1);
    figure_y_size = figure_y_limits(2) - figure_y_limits(1);

    xfactor = screen_width/figure_x_size;
    yfactor = screen_height/figure_y_size;

    if (xfactor < yfactor)
      screen_factor = 0.5*xfactor;
    else
      screen_factor = 0.5*yfactor;
    end

    % calculate screen offsets
    screen_x_offset = (screen_width - screen_factor*figure_x_size)/2;
    screen_y_offset = (screen_height - screen_factor*figure_y_size)/2;

    % draw figure and axes
    set(fig1,'Position', [screen_x_offset screen_y_offset screen_factor*figure_x_size screen_factor*figure_y_size]);
    set(fig1,'MenuBar', 'none');
    axes1 = axes;
    set(axes1,'XLim',figure_x_limits,'YLim',figure_y_limits);
%     set(axes1,'Position',[0 0 1 1]);
%     set(axes1,'Color','w');
    %set(axes1,'TickDir','out');
    axis equal ;
    
    if isrow(torig)
        torig = torig';
    end
    if size(x,2)>size(x,1)
        x = x';
    end
    
    box on;
    [t, x] = even_sample(torig, x, RATE);
    t = t+torig(1);    
    
    xd_flag = false;
    if nargin > 3
        if length(varargin{1}) == 2
            xd_ = varargin{1};
            xd = [xd_(1)*ones(length(x),1), xd_(2)*ones(length(x),1)];
            xd_flag = true;
        end
    end
    if nargin > 4
        [~, xd] = even_sample(varargin{1},varargin{2},RATE);
        xd_flag = true;
    end
    if nargin > 5
        
    end

hist = 50 ;

    for i=1:length(t)
        drawQuadrotor(axes1, x(i,:)');
        
        plot(x(max(1,i-hist):i, 1), x(max(1,i-hist):i, 2), 'k') ;
        if xd_flag
            plot(xd(max(1,i-hist):i, 1), xd(max(1,i-hist):i, 2), 'og','linewidth',12) ;
        end
    %         plot3(x(max(1,i-hist):i, 1)-L*x(max(1,i-hist):i,7), x(max(1,i-hist):i, 2)-L*x(max(1,i-hist):i,8), x(max(1,i-hist):i, 3)-L*x(max(1,i-hist):i,9), 'r') ;
    %         s = sprintf('Running\n t = %1.2fs \n 1/%d realtime speed',t(i), RATE/25);
    %         text(-1.3,2.4,s,'FontAngle','italic','FontWeight','bold');
        drawnow;
        figure_x_limits_ = figure_x_limits+x(i,1);
        figure_y_limits_ = figure_y_limits+x(i,2);
        set(axes1,'XLim',figure_x_limits_,'YLim',figure_y_limits_);
    end

end

function drawQuadrotor(parent,x)
    tem = get(parent,'Children');
    delete(tem);
    
    y = x(1);
    z = x(2);
    phi = x(3);
    
    s.L = 0.175;
    s.R = 0.05;
    
    base = [[s.L; 0], [-s.L; 0], [s.L; s.R], [-s.L; s.R]];
    R = [cos(phi), -sin(phi);
            sin(phi), cos(phi)];
    
    points = [y;z] + R'*base;  
    
    s.qhandle1 = line([points(1,1), points(1,2)], [points(2,1), points(2,2)]); hold on ;
    s.qhandle2 = line([points(1,1), points(1,3)], [points(2,1), points(2,3)]);
    s.qhandle3 = line([points(1,2), points(1,4)], [points(2,2), points(2,4)]);
    
    set(s.qhandle1,'Color','r', 'LineWidth',2);
    set(s.qhandle2,'Color','b', 'LineWidth',2);
    set(s.qhandle3,'Color','b', 'LineWidth',2);
    
    grid on;
    xlabel('Y');
    ylabel('Z');

end