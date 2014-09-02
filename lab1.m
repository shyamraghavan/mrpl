robot = neato('kilo');
leftEncoder = 1;
distance = 0.2;

timeArray = zeros(1,1);
dataArray = zeros(1,1);
dataIndex = 1;

%% Lab 1 Task 1 Move the Robot
TSTART = tic;

leftEncoderStart = leftEncoder;
v = 0.02;

while leftEncoder - leftEncoderStart < distance
    TLOOPBEGIN = tic;
    robot.sendVelocity(v, v)
    pause(0.001)
    leftEncoder = leftEncoder + v * toc(TLOOPBEGIN);
    timeArray(dataIndex) = toc(TSTART);
    dataArray(dataIndex) = leftEncoder;
    dataIndex = dataIndex + 1;
end

robot.sendVelocity(0,0)
pause(1)

%{
while leftEncoder - leftEncoderStart > 0
    TLOOPBEGIN = tic;
    robot.sendVelocity(-v, -v)
    pause(0.001)
    leftEncoder = leftEncoder + (-v) * toc(TLOOPBEGIN);
    timeArray(dataIndex) = toc(TLOOPBEGIN);
    dataArray(dataIndex) = leftEncoder;
    dataIndex = dataIndex + 1;
end
%}

robot.sendVelocity(0,0)
plot(timeArray, dataArray)