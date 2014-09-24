%% PID Controller Setup for distance
% Example command to start test:
%  clear all;close all;clc; pidSetup;PID.run(1000,2)

% rob = robot('sim','Map',1,'Laser');
rob = robot('giga');
pause(2);
PID = pidController(rob);

PID.setPGain(PID,1);
PID.setIGain(PID,0.001);
PID.setDGain(PID,0.4);
PID.setPlotting(true);
global debug
debug = true;