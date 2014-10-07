max = 6;
figure
hold on
plot(-0.5:0.5:3,0.5*ones(1,8),'r')
plot(2*ones(1,8),-0.8:0.2:0.6,'r')
xlim([-.5 3]);
grid on
color = cool(max+1);
n = 1;
for th = 0:pi/max:pi
    curve = cubicSpiral.planTrajectory(2,0.5,th,1);
    plot(curve.poseArray(1,:),curve.poseArray(2,:),'Color',color(n,:));
    n = n+1;
end