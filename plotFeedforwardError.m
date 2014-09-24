function plotFeedforwardError(currentR,targetR)
% Plots the error on he distance between a target disance and a current
% distance.
global data;

if size(data, 2) < 250
    data = [data [currentR;targetR;(currentR-targetR)]];
    
else
    data(:,1:end-1) = data(:,2:end);
    data(:,end) = [currentR;targetR;currentR-targetR]; 
end

set(get(findobj('Tag','feedForwardErrorPlot'),'Children'),...
    'XData',1:size(data,2),...
    'YData',data(3,:));

% set(get(findobj('Tag','feedForwardPlot'),'Children'),...
%     'XData',1:size(data,2),...
%     'YData',data(1,:),'r',...
%     'XData',1:size(data,2),...
%     'YData',data(2,:),'b');

end

