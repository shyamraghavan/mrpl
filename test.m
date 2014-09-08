time = zeros(3,300);

for index = 1:300
    disp(index)
    time(1,index)= rob.encoders.data.header.stamp.secs;
    time(2,index)= rob.encoders.data.header.stamp.nsecs;
    time(3,index)= milli(rob.encoders.data.header.stamp);
    pause(0.01);
end

figure(3),clf;
hold on
secs = subplot(3,1,1);
nsecs = subplot(3,1,2);
milli = subplot(3,1,3);
plot(secs,1:300,time(1,:),'g');
plot(nsecs,1:300,time(2,:),'r');
plot(milli,1:300,time(3,:),'b');