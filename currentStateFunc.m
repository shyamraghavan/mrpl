function currentState = currentStateFunc(plant)
% Determines if currentState is within error of targetState
global initial
persistent encoderStart encoderTime
    
    if initial 
        encoderStart = plant.encoders.left+plant.encoders.right;
        encoderTime = plant.encoders.header.stamp.secs+plant.encoders.header.stamp.nsecs/1E-9;
        initial = false;
        return;
    end
    
    currentState = struct('val',-1,'time',-1);
    
    currentState.val = 0.5*(plant.encoders.left+plant.encoders.right - encoderStart);
    currentState.time = (plant.encoders.header.stamp.secs+plant.encoders.header.stamp.nsecs/1E-9)...
                        - encoderTime;
end
