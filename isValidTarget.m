function [w, valid] = isValidTarget(x, y, v)
% Given the a set of target position coordinates and a initial velocity,
% determines if the robot can go to the coordinates in a single motion and
% returns said value if it can.

    vMax = 0.3;
    W = 0.230;

    wMax = (vMax-v)*2/W

    bearing = atan(y/x);
    halfDist =0.5*sqrt(x^2+y^2);
    
    w = (v/halfDist)*cos(0.5*pi-bearing);

    if abs(w) < wMax
        valid = true;
    else
        valid = false;
    end
    
end

