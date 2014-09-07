%% Create laser plots
laserFig = figure(2);
clf;

%figure config
set(laserFig,'Resize','off',...
             'ToolBar','none',...
             'Menu','none',...
             'NumberTitle','off',...
             'Name','Laser Data',...
             'OuterPosition',[0,50,680,1020]);
         
% sub plots into a 3 tall 2 wide tiling of plots
laserFigPol= subplot(3,2,1:4);
laserFigCart = subplot(3,2,5:6);

%rho and r. r is a placeholder for the laser data.
r = randi(5,1,360);
rho = 1:360;

% polar plot config and plot call
plot(laserFigPol,r.*cos(rho),r.*sin(rho),'+r',0.2.*cos(rho),0.2.*sin(rho),'b');
title(laserFigPol,'Polar representation of Laser data');
axis(laserFigPol,[-6 6 -6 6]);
set(laserFigPol,'Box','on',...
                'XGrid','on',...
                'YGrid','on',...
                'MinorGridLineStyle','-',...
                'XminorGrid','on',...
                'YminorGrid','on',...
                'MinorGridLineStyle',':',...
                'XTick',[-6 -5 -4 -3 -2 -1 0 1 2 3 4 5 6],...
                'YTick',[-6 -5 -4 -3 -2 -1 0 1 2 3 4 5 6],...
                'Tag','laserFigPol');

% cartesian config and plot call
plot(laserFigCart,rho,r,'*r');
title(laserFigCart,'Raw Laser data');
axis(laserFigCart,[0 360 0 6]);
xlabel(laserFigCart,'Angle (deg)');
ylabel(laserFigCart,'Distance (m)');
set(laserFigCart,'Box','on',...
                 'XTick',[0 30 60 90 120 150 180 210 240 270 300 330 360],...
                 'Tag','laserFigCart');