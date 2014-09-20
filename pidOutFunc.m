function pidOutFunc(pid, controlSig)
%UNTITLED10 Summary of this function goes here
%   Detailed explanation goes here
    vMax = 0.2;
    maxSig = 10;
    ratio = 0.5;

    if controlSig > maxSig
        pid.maxSig = controlSig;
    elseif controlSig < -maxSig
        pid.maxSig = controlSig;
    end

    velocity = vMax*ratio*controlSig;
    disp(velocity);
    pid.plant.velocityControl(velocity,0);
end

