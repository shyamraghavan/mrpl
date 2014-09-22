%% Robot Setup
rob = neato('kilo');

%% Lab 1 Task 1 Move the Robot
    
%declare logging variables
timeArray = zeros(1,1); 
dataArray = zeros(1,1);
dataIndex = 1;

leftEncoderStart = rob.encoders.data.left; %start value of encoder in mm
leftEncoderStartTime = milli(rob.encoders.data.header.stamp); %start time of encoder in milliseconds

v = 0.1;%m Speed setting
distance = 200;%mm Distance setting
pauseTime = 2E3;%s Pause duration between forward and backwards movement in milliseconds

% forward movement loop
while rob.encoders.data.left - leftEncoderStart < distance
    rob.sendVelocity(v, v);
    pause(0.01);

    %Calculate changes in dist and time from the values at the start of the
    %movement.
    deltaLeftEncoderDist = rob.encoders.data.left - leftEncoderStart;
    deltaLeftEncoderTime = milli(rob.encoders.data.header.stamp) - leftEncoderStartTime;
   
    %Logs the changes in the appropriate variables at the appropriate
    %indexes.
    timeArray(dataIndex) = deltaLeftEncoderTime;
    dataArray(dataIndex) = deltaLeftEncoderDist;
    dataIndex = dataIndex + 1;
    disp('forward')
end
disp('pause')
% Pause loop
pauseEnd = milli(rob.encoders.data.header.stamp) + pauseTime; %start of pause
while milli(rob.encoders.data.header.stamp) < pauseEnd
    rob.sendVelocity(0,0)
    pause(0.01)
    
    %Calculate changes in dist and time from the values at the start of the
    %movement.
    deltaLeftEncoderDist = rob.encoders.data.left - leftEncoderStart;
    deltaLeftEncoderTime = milli(rob.encoders.data.header.stamp) - leftEncoderStartTime;
    
    %Logs the changes in the appropriate variables at the appropriate
    %indexes.
    timeArray(dataIndex) = deltaLeftEncoderTime;
    dataArray(dataIndex) = deltaLeftEncoderDist;
    dataIndex = dataIndex + 1;
    
    disp(milli(rob.encoders.data.header.stamp) - pauseTime)
end

while rob.encoders.data.left - leftEncoderStart > 0
    rob.sendVelocity(-v, -v);
    pause(0.01);
    
    %Calculate changes in dist and time from the values at the start of the
    %movement.
    deltaLeftEncoderDist = rob.encoders.data.left - leftEncoderStart;
    deltaLeftEncoderTime = milli(rob.encoders.data.header.stamp) - leftEncoderStartTime;
    
    %Logs the changes in the appropriate variables at the appropriate
    %indexes.
    timeArray(dataIndex) = deltaLeftEncoderTime;
    dataArray(dataIndex) = deltaLeftEncoderDist;
    dataIndex = dataIndex + 1;
    disp('backward')
end

rob.sendVelocity(0,0);% stop robot.
plot(timeArray, dataArray);% plot logged values of dist vs. time