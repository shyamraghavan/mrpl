function plotDistanceError(currentR,targetR)
% Plots the error on he distance between a target disance and a current
% distance.
global error;
error = [error currentR-targetR];
x = 1:size(error,2);
set(get(findobj('Tag','errorPlot'),'Children'),...
    'XData',x,...
    'YData',error);

end

