%% PID Controller Setup for distance

rob = robot('sim','Map',1,'Laser');

PID = pidController(rob);

setPGain(PID,5);
setIGain(PID,0.1);
setDGain(PID,0.1);
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