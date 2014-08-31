classdef robotKinematicModel < handle
    %robotKinematicModel A convenience class for storing robot physical 
    % and performing related kinematic transforms. You can reference the
    % defined constants via the clas name with robotKinematicModel.W2 for
    % example because they are constant properties and therefore associated
    % with the class rather than any instance. Similiarly, the kinematics
    % routines are referenced from the class name as well.
    
    properties(Constant)
        W  = 9.25*2.54/100;   % wheel tread in m
        W2 = 9.25*2.54/2/100; % 1/2 wheel tread in m
    end
    
    properties(Access = private)
    end
    
    properties(Access = public)
    end
    
    methods(Static = true)
        
        function [V w] = vlvrToVw(vl, vr)
        % Converts wheel speeds to body linear and angular velocity.
            V = (vr + vl)/2.0;
            w = (vr - vl)/robotKinematicModel.W;
        end
        
        function [vl vr] = VwTovlvr(V, w)
        % Converts body linear and angular velocity to wheel speeds.
            vr = V + robotKinematicModel.W*w;
            vl = V - robotKinematicModel.W*w;
        end
        
    end
    
    methods(Access = private)
        
    end
            
    methods(Access = public)
        

    end
end