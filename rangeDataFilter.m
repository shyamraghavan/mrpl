function [endData] = rangeDataFilter(rawData)
%Takes in raw LIDAR data and filters the data according to some heuristics
%discribed below. In addition it calculates a confidence weight to each
%data point from 0 to 1
confidence = ones(1,360);
MAXRANGE = 5;
MINRANGE = 0.1;

% band pass the data between MINRANGE and MAXRANGE.
temp = rawData.*(rawData < MAXRANGE);
confidence = confidence .*(rawData < MAXRANGE);
endData = temp .* (temp>MINRANGE);
confidence = confidence .*(temp>MINRANGE);

end

