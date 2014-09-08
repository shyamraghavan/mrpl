function plotDistanceError(currentR,targetR)
% Plots the error on he distance between a target disance and a current
% distance.
global error;

if size(error, 2) < 250
    error = [error currentR-targetR];
else
    error(1:end-1) = error(2:end);
    error(end) = currentR-targetR;
end
x = 1:size(error,2);
set(get(findobj('Tag','errorPlot'),'Children'),...
    'XData',x,...
    'YData',error);

end

