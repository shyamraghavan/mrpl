classdef pidController < handle
    %  /!\                                                        /!\  %
    % / ! \ Careful with units! all of this code is in mm not cm / ! \ %
    %/__!__\                                                    /__!__\%
    properties
        
        % /!\ Never initialize in the properties: it will cause aliasing
        % problems if there is more than one instance.
        robot;
        kp;
        ki;
        kd;

    end
    methods
        
        function obj = pidController(robot)
            obj.robot = robot;
        end
        
        function setGain(obj,kp,ki,kd)
            obj.kp = kp;
            obj.ki = ki;
            obj.kd = kd;
        end
        
        % Feedback only
        function runPID(obj, targetState, margin)
            % target state is a struct with two fields, x and y. these are
            % expressed in the robot's coordinate system.
            
            % Checks for unreasonable input
            if margin == 0
                error('CANNOT HAVE ZERO ERROR.');
            end
            
            % finds the curvature and the total arclenght
            k = findCurvature(targetState.x,targetState.y);
            distance = (2/k)*asin(k*sqrt(x^2+y^2));
           
            
%             currentState = struct('dist',-1,'time',-1);
            currentState = getCurrentState(true);
            % currentState is struct with fields dist and time. Calling
            % getCurrentState with the argument true zeroes out the
            % different encoders on the robot greatly simplifying the math.
            
            integralError = 0;
            previousError = struct('dist',0,'time',-1);
            %zeroes the running error integral and the derivative memory.
            while ~withinMargin(distance,currentState.dist, margin)
                % this is the actual PID control loop. While our travelled
                % distance is not within the given margin with zero
                % velocity, continue to apply corrective signals.
                
                proportionalGain = obj.kp*(distance-currentState.dist);
                
                integralGain = obj.ki*((distance-currentState.dist)+integralError);
                
                % We need at least two datapoints to extract a derivative,
                % thus we ignore the first loop iteration.
                if(previousError.time == -1)% first iteration
                    derivativeGain = 0; 
                else% subsequent iteration
                    derivativeGain = obj.kd*((distance-currentState.dist)-previousError.dist)/...
                                            (previousError.time-currentState.time);
                end
                previousError.dist = (distance-currentState.dist);
                previousError.time = currentState.time;    
                % Update the previous error.
                
                commandRobot(proportionalGain+integralGain+derivativeGain, k);
                currentState = getCurrentState(false);
            end
            
            commandRobot(0);
        end
        
        % Feedforward
        function runFF(obj, goalPose, sgn, margin)
            % Checks for unreasonable input
            if margin == 0
                margin('CANNOT HAVE ZERO ERROR.');
            end 
            
            curve = cubicSpiral.planTrajectory(goalPose(1),goalPose(2),goalPose(3),sgn);
            curve.planVelocities(0.25);%Vmax minus some wiggle room for the controller.
            
            while (currentState.time <= curve.getTrajectoryDuration) ||...
                    ~withinMargin(curve.getTrajectoryDistance,currentState.dist, margin)
                %
                %
                % stuff
                %
                %
            end
        end
        
        % Utility Functions
        function currentState = getCurrentState(initial,~)
            persistent encoderL encoderR
            if initial
                encoderL = obj.robot.currLeft;
                encoderR = obj.robot.currRight;
            end
            switch nargin
                case 1 % feedback only
                    currentState.dist = ((obj.robot.currLeft -encoderL)+...
                                         (obj.robot.currRight-encoderR))/2;%mm
                    currentState.time = obj.robot.currTime;
                    return;
                case 2 % feedfordward
                    currentState.distL = obj.robot.currLeft  -encoderL;%mm
                    currentState.distR = obj.robot.currRight -encoderR;%mm
                    currentState.time = obj.robot.currTime;
                    return;
                otherwise % invalid number of arguments
                    error('Invalid number of arguments');
            end
        end
        
        function valid = withinMargin(target,dist, margin)
            if abs(target-dist)<margin 
            % within margin of the target dist.
                if obj.robot.velocity < 0.01
                % within acceptable limits of zero speed.
                    valid = true;
                    return;
                end
            end
            valid = false;
        end
        
        function commandRobot(arg1,arg2,arg3)
            switch nargin
                case 1 %commadRobot(0)
                    % We command the robot to stop
                    obj.robot.velocityControl(0,0);
                    return;
                    
                case 2 %commandRobot(controlSig,k)
                    % We command the robot using only feedback control
                    controlSig = arg1;
                    k = arg2;
                    % We check to make sure the velocities are acceptable.
                    if controlSig > 300%mm/s
                        v = 300;
                    elseif controlSig < -300%mm/s
                        v = -300;
                    else
                        v = controlSig;
                    end
                    
                    obj.robot.velocityControl(v/1000,v*k/1000);%cm/s
                    return;
                    
                case 3 %commandRobot(controlSig,Vref,Wref)
                    % We command the robot using feedforward and feedback
                    controlSig = arg1;
                    Vref = arg2;
                    Wref = arg3;
                otherwise
                    error('Incorect number of inputs.');
            end
        end
    end
end