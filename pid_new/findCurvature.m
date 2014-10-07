function [k] = findCurvature(x, y)
% For a given x and y, finds the curvature of the circle that is both
% tangeant to the point (0,0) and (x,y).

bearing = atan(y/x);
halfDist =0.5*sqrt(x^2+y^2);

k = cos(0.5*pi-bearing)/halfDist;

end

