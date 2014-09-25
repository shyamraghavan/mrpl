function plotOutput(vff,vel)
% Plots the error on he distance between a target disance and a current
% distance.
global data;

data(4:5,end) = [vff;vel];

set(findobj('Tag','vffPlot'),...
    'XData',1:size(data,2),...
    'YData',data(4,:));
set(findobj('Tag','velPlot'),...
    'XData',1:size(data,2),...
    'YData',data(5,:));
end

