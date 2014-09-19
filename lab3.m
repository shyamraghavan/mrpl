close all;
clear all;
clc;
%% Robot Setup
rob = robot('sim','Laser');

% Setup laser plot update lisener
run('laserFigConfig.m');
laserFigUpdateListener = event.listener(rob.neatoRobot.laser,'OnMessageReceived',@onNewLaserData);

% Filter laser data
cleanedLaserRanges = rangeDataFilter(rob.neatoRobot.laser.data.ranges);

% Detect nearest object within a specified sector
[i,r] = nearestObject(inSector(cleanedLaserRanges, -45, 45));

posePlot = plot(rob.xPositions, rob.yPositions, 'b-');
set(posePlot, 'Tag', 'posePlot');
xlim([0.0 0.5]);
ylim([0.0 0.5]);
             
k = 0.314;
index = 1;
tic
while toc < 9
    if index >= 2
        rob.theta = rob.theta + rob.omega * rob.dt;
        rob.xPositions(index) = rob.xPositions(index - 1) + rob.v * cos(rob.theta) * rob.dt;
        rob.yPositions(index) = rob.yPositions(index - 1) + rob.v * sin(rob.theta) * rob.dt;
        vr = 0.2+(1/10)*sin(toc*k);
        vl = 0.2-(1/10)*sin(toc*k);
    else
        vr = 0;
        vl = 0;
    end
    set(posePlot, 'xdata', rob.xPositions, ...
        'ydata', rob.yPositions);
    
    index = index + 1;

    rob.neatoRobot.sendVelocity(vr,vl);
end
% while toc < 6.5
%     rob.neatoRobot.sendVelocity(0.2,0.1)
% end
% 
% while toc < 15.3
%     rob.neatoRobot.sendVelocity(0.1,0.2)
% end
pause(5);

if 0
disp('Launching Error Plot.');
% Set up error plot
global error;
error = [];
figure(3);
errorPlot = subplot(1,1,1);
plot(errorPlot,1:10,1:10,'+m');
title(errorPlot,'Distance Error (m)');
set(errorPlot,'Tag','errorPlot',...
              'YMinorGrid', 'on',...
              'YLim',[-1 1]);
pause(3);

% Velocity and turn speed variables
v = 0.15;
differential = 0.08;

while rob.robotActive
    
     plotDistanceError(r,0.65); % plot new error distance
     
    % Determine action based on range and bearing.
    if  i == -1 || r > 2
        rob.neatoRobot.sendVelocity(0,0);
    elseif r > 0.70
%         plotDistanceError(r,0.65); % plot new error distance
        if i < 45 && i > 10
            rob.neatoRobot.sendVelocity(v - differential, v + differential);
        elseif i > 315 && i < 350
            rob.neatoRobot.sendVelocity(v + differential, v - differential);
        else
            rob.neatoRobot.sendVelocity(v,v);
        end
    elseif r < 0.60
%         plotDistanceError(r,0.65); % plot new error distance
        rob.neatoRobot.sendVelocity(-v,-v);
    else
%         plotDistanceError(r,0.65); % plot new error distance
        rob.neatoRobot.sendVelocity(0,0);
    end
    
    % Update bearing and range to target.
    cleanedLaserRanges = rangeDataFilter(rob.neatoRobot.laser.data.ranges);
    [i,r] = nearestObject(inSector(cleanedLaserRanges, -45, 45));
    
    pause(0.05);
end
end
%%
rob.close
