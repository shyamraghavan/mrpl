function plotFeedforwardError(currentR,targetR,error)
% Plots the error on he distance between a target disance and a current
% distance.
global data;

if size(data, 2) < 1000
    data = [data [currentR;targetR;error;0;0]];
    
else
    data(:,1:end-1) = data(:,2:end);
    data(:,end) = [currentR;targetR;error;0;0]; 
end

set(get(findobj('Tag','feedForwardErrorPlot'),'Children'),...
    'XData',1:size(data,2),...
    'YData',data(3,:));

set(findobj('Tag','feedForwardPlot1'),...
    'XData',1:size(data,2),...
    'YData',data(1,:));
set(findobj('Tag','feedForwardPlot2'),...
    'XData',1:size(data,2),...
    'YData',data(2,:));

end

