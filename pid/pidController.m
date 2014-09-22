classdef pidController < handle
    properties
        % type of controler: PID, PD, PI,....
        type = 'pid';
        
        % Gains
        kp = 0;
        ki = 0;
        kd = 0;

        
        currentState = struct('val',-1,...
                              'time',-1);
        plant;
        
        % Variables required for the integral and differential error terms
        previousError = struct('val',-1,...
                               'time',-1);
        %proportional gain the first time around the loop?
        integralError=0;
    end
    methods
        %% Constructor
        function obj = pidController(plantObj)
            obj.plant = plantObj;
        end       
        
        %% Setters for the different gains
        function setPGain(obj,k)
            obj.kp = k;
        end
        
        function setIGain(obj,k)
            obj.ki = k;
        end
        
        function setDGain(obj,k)
            obj.kd = k;
        end
              
        %% Run is the main control loop of the controller
        function run(obj,targetState, error)
            print('Running');
            output = [0,0,0];
	    % Checks for unreasonable input
            if error == 0
                error('CANNOT HAVE ZERO ERROR.');
            end
            
	    % Running the currentStateFunc with a true argument is required to zero out the initial values of the encoders
            obj.currentState = obj.currentStateFunc(true);
            while ~obj.epsilonFunc(targetState, error)
                print('start')
                print(obj);
		% Proportional gain
                output(1) = (targetState - obj.currentState.val)*obj.kp;
                print(output(1));
              	
		% Integral Gain
		output(2) = ((targetState - obj.currentState.val)...
                            + obj.integralError)*obj.ki;
                print(output(2));
                
		if obj.previousError.time == -1
                    obj.previousError.val = (targetState - obj.currentState.val);
                    obj.previousError.time = obj.currentState.time;
                    output(3) = 0;
                else
                    output(3) = ...
                    (((targetState - obj.currentState.val) - obj.previousError.val)/...
                    (obj.currentState.time -obj.previousError.time))*obj.kd;
                    obj.previousError.time = obj.currentState.time;
                    obj.previousError.val  =(targetState - obj.currentState.val);
                end
                print(output(3));
                
		plotDistanceError(obj.currentState, targetState);
            	obj.pidOutFunc(sum(output))
           	obj.currentState = obj.currentStateFunc(false);

            end 
            obj.plant.velocityControl(0,0);
            obj.integralError = 0;
            obj.previousError.time = -1;
            
        end
        
        function currentState = currentStateFunc(obj,initial)
            % Determines currentState
	    % These persistent values tare the currentState
            persistent encoderStart encoderTime
                if initial 
                    encoderStart = obj.plant.encoders.left+obj.plant.encoders.right;
                    encoderTime = obj.plant.encoders.header.stamp.secs+obj.plant.encoders.header.stamp.nsecs*1E-9;
                end
                currentState.val = 0.5*(obj.plant.encoders.left+obj.plant.encoders.right - encoderStart);
                currentState.time = (obj.plant.encoders.header.stamp.secs+obj.plant.encoders.header.stamp.nsecs*1E-9)...
                                    - encoderTime;
        end
        
        function valid = epsilonFunc(obj, targetState, error)
            % Determines if currentState is within error of targetState
	    % if we are not within boundaries we short circuit the evaluation.
            if abs(obj.currentState.val - targetState)>error
                valid = false;
                return;
            else
		% else we check to see if the velocity is indeed within error margin.
                if abs(obj.plant.velocity) < 0.01
                    valid = true;
                    return
                end
                valid = false;
            end
        end

        function pidOutFunc(obj, controlSig)
            % Outputs our control signal to the robot.
            vMax = 0.2;
            ratio = 0.5;
       
            velocity = vMax*ratio*controlSig*0.001;
            if velocity > 0.3
                velocity = 0.3;
            elseif velocity <-0.3
                velocity = -0.3;
            end
            obj.plant.velocityControl(velocity,0);
            pause(0.5);
        end
    end
end