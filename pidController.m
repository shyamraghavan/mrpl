classdef pidController < handle
    properties
        kp = 0;
        ki = 0;
        kd = 0;
       
        targetState;
        previousState;
        
        integralError;
    end
    methods
        function obj = pidController()
        end
        
        function setTargetState(obj,state)
            obj.targetState = state;
        end
        
        function resetIntegralError(obj)
            obj.integralError = 0;
        end
        
        function setDistancePID(obj,dist,margin)
            
        end
        
    end
end