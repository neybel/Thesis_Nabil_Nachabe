function waypoints = Way()
ocp_N = 100;
x1=110*ones(1,ocp_N+1);
y1=linspace(-20,-10,ocp_N+1);
%% 
    theta = linspace(0, pi/4, 50+ocp_N); % Angle for the arc
     radius = 15;
    centerX = 95; % Center of the arc
    centerY = -10;
    x2 = centerX + radius * cos(theta);
    y2 = centerY + radius * sin(theta);
    x3 = linspace(x2(end),x2(end)-40,150+ocp_N);
    y3 = linspace(y2(end),y2(end),150+ocp_N);
    X=[x1,x2,x3];
    Y=[y1,y2,y3];
    waypoints= [X;Y];
%     hold on 
%     plot(X,Y,'k--');
end
