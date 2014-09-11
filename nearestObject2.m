function [index] = nearestObject2(ranges, tolerance)
% returns the index of the closest object that is non zero, less than
% maxObjectRange and within +/- pi/3 of the direction of travel of the
% robot. returns -1 if no index satisfies requirements.

maxObjectRange = 2.5;%m

 ranges(~ranges(:)) = 10;
 minValue = min(ranges);
 
 index = find(ranges < (minValue + tolerance));
 
 if minValue >= maxObjectRange
    index = -1;
 end
end

