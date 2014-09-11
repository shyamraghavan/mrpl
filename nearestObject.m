function [minIndex,minValue] = nearestObject(ranges)
% returns the index of the closest object that is non zero, less than
% maxObjectRange and within +/- pi/3 of the direction of travel of the
% robot. returns -1 if no index satisfies requirements.

 maxObjectRange = 2.5;%m

 [minValue,minIndex] = min(ranges);
 
 if minValue >= maxObjectRange
    minIndex = -1;
 end
end

