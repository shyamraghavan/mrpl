classdef Movement < handle
    %Movement Class to encapsulate all information associated with movement
    
    properties (SetAccess = private)
        robot
        wheelbase = 0.2350;
    end
    
    methods
        function obj = Movement(robot, wheelbase)
            obj.robot = robot;
            obj.wheelbase = wheelbase;
        end
        
        function velocityControl(obj, v, omega)
            vr = v + obj.wheelbase / 2 * omega;
            vl = v - obj.wheelbase / 2 * omega;
            obj.robot.sendVelocity(vr, vl);
        end
        
        function goToXY(obj, deltaX, deltaY, deltatheta, velocity)
            kappa = deltatheta / sqrt(deltaX^2 + deltaY^2);
            omega = kappa * velocity;
            obj.velocityControl(velocity, omega);
        end
    end
end

