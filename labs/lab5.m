clear all
close all
clc

rob = robot('sim');

refControl = ReferenceControl();
refControl.figure8(.5,.5,1)

plot(refControl.v)
hold on
plot(refControl.w)

for t = 0:0.05:refControl.getTrajectoryDuration()
    [V,W] = refControl.computeControl(t);
    rob.velocityControl(V,W);
    pause(0.05)
end