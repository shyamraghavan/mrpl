%% PID Controller Setup for distance
% Example command to start test:
%  clear all;close all;clc; pidSetup;PID.run(1000,2)

% rob = robot('sim','Map',1,'Laser');
rob = robot('giga');
pause(2);
PID = pidController(rob);

setPGain(PID,1);
setIGain(PID,0);
setDGain(PID,0);
PID.setPlotting(true);
global debug max min
debug = true;
max = 0;
min = 0;