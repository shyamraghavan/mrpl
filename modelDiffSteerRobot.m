function [x,y,theta] = modelDiffSteerRobot(vl,vr,t0,tf,dt)
% For given velocities, in mm/sec, returns the robot's trajectory.

wheelbase = 0.2350;m

t = t0:dt:tf;
k = length(t);
x = zeros(1,k);
y = zeros(1,k);
theta = zeros(1,k);

v = (vl+vr)/2;
w = (vl-vr)/wheelbase;

for n = 1:k
    
    
    
    theta(n) = theta(n-1)+w*dt*0.5
    deltax = v*cos(theta)*dt
    deltay = v*sin(theta)*dt
    deltatheta = theta+w*dt*0.5
end

end

