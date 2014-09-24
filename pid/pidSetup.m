%% PID Controller Setup for distance
% Example command to start test:
%  clear all;close all;clc; pidSetup;PID.run(1000,2)

rob = robot('mega');
pause(2);
PID = pidController(rob);

setPGain(PID,1);
setIGain(PID,0.001);
setDGain(PID,0.4);
global debug
debug = true;
disp('Launching Error Plot.');
% Set up error plot
global error;
error = [[0];[0]];
figure(3);
errorPlot = subplot(1,1,1);
plot(errorPlot,error(2),error(1),'+m');
title(errorPlot,'Distance Error (m)');
set(errorPlot,'Tag','errorPlot',...
              'YMinorGrid', 'on'              );