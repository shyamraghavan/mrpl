function [x, y, b] = irToXy(i,r)
% Converts a range (r) at index (i) to an (x,y) position relative to the 
% robotin meters as well as a bearing in degrees from the x axis of the 
% robot, a.k.a the direction of motion of the robot.

b = i*pi/180;
x = r*cos(b);
y = r*sin(b);

end

