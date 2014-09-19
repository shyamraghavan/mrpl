close all;
clear all;
clc;
%% Robot Setup
rob = robot('sim', 'Laser');

pause(5);

while true
    rob.neatoRobot.sendVelocity(0.1,0.1);
end








if 0
% Setup laser plot update lisener
run('laserFigConfig.m');
laserFigUpdateListener = event.listener(rob.neatoRobot.laser,'OnMessageReceived',@onNewLaserData);

% Filter laser data
cleanedLaserRanges = rangeDataFilter(rob.neatoRobot.laser.data.ranges);

% Detect nearest object within a specified sector
[i,r] = nearestObject(inSector(cleanedLaserRanges, -45, 45));

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
% rob.close
