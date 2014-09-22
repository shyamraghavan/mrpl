function onNewEncoderData(src,evt,robotClass)
    robotClass.encoders = evt.data;
    
    robotClass.currTime = evt.data.header.stamp.secs + evt.data.header.stamp.nsecs*1E-9;
    robotClass.currLeft = evt.data.left;
    robotClass.currRight = evt.data.right;
    if robotClass.prevTime ~= -1
        robotClass.dt = robotClass.currTime - robotClass.prevTime;
%         disp('dt ');
%         disp(dt);
    end
    if robotClass.prevLeft ~= -1
        robotClass.dLeft = robotClass.currLeft - robotClass.prevLeft;
%         disp('dLeft ');
%         disp(dLeft);
    end
    if robotClass.prevRight ~= -1
        robotClass.dRight = robotClass.currRight - robotClass.prevRight;
%         disp('dRight ');
%          disp(dRight);
    end
    
    robotClass.vLeft = robotClass.dLeft / robotClass.dt;
    robotClass.vRight = robotClass.dRight / robotClass.dt;
    % Velocity of left wheel + right wheel / 2, and then convert into
    % meters per second.
    robotClass.velocity = (robotClass.vLeft + robotClass.vRight) / (2 * 1000);
%     disp(robotClass.velocity);
    robotClass.prevTime = evt.data.header.stamp.secs + evt.data.header.stamp.nsecs*1E-9;
    robotClass.prevLeft = robotClass.currLeft;
    robotClass.prevRight = robotClass.currRight;
    
end