function [ output_args ] = goTo(rob, xf, yf, thf, sign)
%GOTO Summary of this function goes here
%   Detailed explanation goes here

%     totalError = 0;
    x = 0;
    y = 0;
    actualTheta = 0;
    actualX = [0];
    actualY = [0];
    maxVelocity = 0.25;
    KpLinear = 0.01;
    KpAngular = 0.05;

    curve = cubicSpiral.planTrajectory(xf, yf, thf, sign);
    curve.planVelocities(maxVelocity);
    
%     initialX = [0];
%     initialY = [0];
%     initialTheta = actualTheta;
    
    poseArray(1,:) = 200 * xf / curve.poseArray(1,end) * curve.poseArray(1,:);
    poseArray(2,:) = 200 * yf / curve.poseArray(2,end) * curve.poseArray(2,:);
    
    encoderLeft = rob.encoders.data.left / 1000;
    encoderRight = rob.encoders.data.right / 1000;
    
    actualLeftVelocity = 0;
    actualRightVelocity = 0;
    
    robotTime = rob.encoders.data.header.stamp.secs + rob.encoders.data.header.stamp.nsecs*1E-9;
    
%     pause(0.05);
    
    timeArray = curve.timeArray;
    velocityArray = curve.VArray;
    angularVelocityArray = curve.wArray;
    
    initialTime = tic();
    
    stop = false;
    
    while true && ~stop
        currentTime = toc(initialTime);
        
        if currentTime >= curve.getTrajectoryDuration()
            stop = true;
            rob.sendVelocity(0,0);
            break;
        end
        
        velocity = interp1(timeArray, velocityArray, currentTime);
        angularVelocity = interp1(timeArray, angularVelocityArray, currentTime);
        
        newRobotTime = rob.encoders.data.header.stamp.secs + rob.encoders.data.header.stamp.nsecs*1E-9;
        dt = newRobotTime - robotTime;
        robotTime = newRobotTime;
        
        newEncoderLeft = rob.encoders.data.left / 1000;
        newEncoderRight = rob.encoders.data.right / 1000;
        if (newEncoderLeft - encoderLeft) / dt <= 0.3 && (newEncoderLeft - encoderLeft) / dt >= 0
            actualLeftVelocity = (newEncoderLeft - encoderLeft) / dt;
            actualRightVelocity = (newEncoderRight - encoderRight) / dt;
        end
        encoderLeft = newEncoderLeft;
        encoderRight = newEncoderRight;
        
        actualVelocity = (actualLeftVelocity + actualRightVelocity) / 2;
        actualAngularVelocity = (actualRightVelocity - actualLeftVelocity) / robotModel.W;
        
        actualTheta = actualTheta + actualAngularVelocity * dt;
        x = x + actualVelocity * cos(actualTheta) * dt;
        y = y + actualVelocity * sin(actualTheta) * dt;
        
        actualX = [actualX x];
        actualY = [actualY y];
        
        distanceToPoint = norm([interp1(timeArray, poseArray(1,:), currentTime)-actualX(end),...
            interp1(timeArray, poseArray(2,:), currentTime)-actualY(end)]);
        angleToPoint = atan2(interp1(timeArray, poseArray(1,:), currentTime)-actualX(end),...
            interp1(timeArray, poseArray(2,:), currentTime)-actualY(end));
        
        if (distanceToPoint >= 0.1)
            velocity = velocity + KpLinear * distanceToPoint;
        end
        if (angleToPoint >= 0.05)
            angularVelocity = angularVelocity + KpAngular * angleToPoint;
        end
        
        leftVelocity = velocity - robotModel.W2 * angularVelocity;
        rightVelocity = velocity + robotModel.W2 * angularVelocity;
        
        if leftVelocity > 0.3 || rightVelocity > 0.3
            scale = 0.29 / max(leftVelocity, rightVelocity);
            leftVelocity = scale * leftVelocity;
            rightVelocity = scale * rightVelocity;
        end
        
        rob.sendVelocity(leftVelocity, rightVelocity);
        pause(0.01);
    end
%     totalError = totalError + norm([poseArray(1,end)-actualX(end),poseArray(2,end)-actualY(end)]);  
%     currentPose = pose(initialX, initialY, initialTheta);
%     graphPoseArray = currentPose.bToA*[poseArray(1,:);poseArray(2,:);ones(1,201)];
end

