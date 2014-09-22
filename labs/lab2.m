close all;
clear all;
clc;
%% Robot Setup
rob = neato('giga');
pause(1);

%% Lab 2 - Smart Luggage
% Start laser rangefinder
rob.startLaser();
pause(2);
disp('Robot''s laser is activated.');

% Setup laser plot update lisener
run('laserFigConfig.m');
laserFigUpdateListener = event.listener(rob.laser,'OnMessageReceived',@onNewLaserData);

% Filter laser data
cleanedLaserRanges = rangeDataFilter(rob.laser.data.ranges);

% Detect nearest object within a specified sector
[i,r] = nearestObject(inSector(cleanedLaserRanges, -45, 45));

disp('Launching Error Plot.');5
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

while true
    
     plotDistanceError(r,0.65); % plot new error distance
     
    % Determine action based on range and bearing.
    if  i == -1 || r > 2
        rob.sendVelocity(0,0);
    elseif r > 0.70
%         plotDistanceError(r,0.65); % plot new error distance
        if i < 45 && i > 10
            rob.sendVelocity(v - differential, v + differential);
        elseif i > 315 && i < 350
            rob.sendVelocity(v + differential, v - differential);
        else
            rob.sendVelocity(v,v);
        end
    elseif r < 0.60
%         plotDistanceError(r,0.65); % plot new error distance
        rob.sendVelocity(-v,-v);
    else
%         plotDistanceError(r,0.65); % plot new error distance
        rob.sendVelocity(0,0);
    end
    
    % Update bearing and range to target.
    cleanedLaserRanges = rangeDataFilter(rob.laser.data.ranges);
    [i,r] = nearestObject(inSector(cleanedLaserRanges, -45, 45));
    
    pause(0.05);
end

%%
rob.stopLaser();
rob.close();
