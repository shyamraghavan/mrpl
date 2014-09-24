classdef pidController < handle
    properties
        % type of controler: PID, PD, PI,....
        type = 'pid';
         
        plotting = false;
        % Gains
        kp = 0;
        ki = 0;
        kd = 0;

        
        currentState = struct('val',-1,...
                              'time',-1);
        perceivedState = struct('val',-1,...
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
        
        function setPlotting(obj,valid)
            controlFigConfig();
            obj.plotting = valid;
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
        
        function setPIDGain(obj,k1,k2,k3)
            obj.kp = k1;
            obj.ki = k2;
            obj.kd = k3;
        end
              
        %% Run is the main control loop of the controller
        function runPID(obj,targetState, error)
            print('Running');
            output = [0,0,0];
	    % Checks for unreasonable input
            if error == 0
                error('CANNOT HAVE ZERO ERROR.');
            end
            
	    % Running the currentStateFunc with a true argument is required to zero out the initial values of the encoders
            obj.currentState = obj.currentStateFunc(true);
            while ~obj.epsilonFunc(targetState, error)
                
                % Proportional gain
                output(1) = (targetState - obj.currentState.val)*obj.kp;
                
                % Integral Gain
                output(2) = ((targetState - obj.currentState.val)...
                            + obj.integralError)*obj.ki;
                % Update integral error
                obj.integralError = obj.integralError + (targetState - obj.currentState.val);
                
                % Derivative Gain
                if obj.previousError.time == -1 
                    % edge case when we have no derivative gain.
                    %update previous error
                    obj.previousError.val = (targetState - obj.currentState.val);
                    obj.previousError.time = obj.currentState.time;
                    output(3) = 0;
                else
                    output(3) = ...
                    (((targetState - obj.currentState.val) - obj.previousError.val)/...
                    (obj.currentState.time -obj.previousError.time))*obj.kd;
                    %update previous error
                    obj.previousError.time = obj.currentState.time;
                    obj.previousError.val  =(targetState - obj.currentState.val);
                end

                print('Current Error:\t%dmm\n',targetState - obj.currentState.val);    
                if obj.plotting
                    plotDistanceError(obj.currentState.val, targetState);
                end
                
                %creates control signal and gives it to pidOutFunc for
                %actual output.
                obj.pidOutFunc(sum(output),0)
                %update current state
                obj.currentState = obj.currentStateFunc(false);
            end 
            
            print('DONE');
            % stops robot
            obj.plant.velocityControl(0,0);
            %reset errors
            obj.integralError = 0;
            obj.previousError.time = -1;
            
        end
        
        %% Run with Feed Forward
        function runFF(obj,targetState, error)
            print('Running');
            output = [0,0,0];
            
	    % Checks for unreasonable input
            if error == 0
                error('CANNOT HAVE ZERO ERROR.');
            end    
	    % Running the currentStateFunc with a true argument is required to zero out the initial values of the encoders
            vmax = 0.25;
            amax = 3*0.25;
            dist = targetState/1000;
            vFF = trapezoidalVelocityProfile(0,amax,vmax,dist);
            
            obj.currentState = obj.currentStateFunc(true);
            obj.plant.velocityControl(vFF,0);

            time = tic; % start time for integrator
            expectedDist = 0; %expected displacement
            while ~obj.epsilonFunc(targetState, error)
                
                lastTime = time;
                currentTime = toc(time);
                dt = currentTime - lastTime;
                
                % current velocity value acording to our profile
                vFF = trapezoidalVelocityProfile(currentTime,amax,vmax,dist);
                
                if ~obj.epsilonFunc(expectedDist,error)
                    % Proportional gain
                    output(1) = (targetState - obj.currentState.val)*obj.kp;

                    % Integral Gain
                    output(2) = ((targetState - obj.currentState.val)...
                                + obj.integralError)*obj.ki;
                    obj.integralError = obj.integralError + (targetState - obj.currentState.val);

                    % Derivative Gain
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
                end
                
                expectedDist = expectedDist + vFF * dt;
                obj.pidOutFunc(sum(output),vFF);
                if obj.plotting
                    plotFeedforwardError(expectedDist*1000,obj.currentState.val)
                end
                pause(0.01);
            end
            print('DONE');
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
        
        function pidOutFunc(obj, controlSig,velocity)
            % Outputs our control signal to the robot.       
            trim = controlSig*0.001;
            if velocity+trim > 0.3
                velocity = 0.3;
            elseif velocity+trim <-0.3
                velocity = -0.3;
            else
                velocity = velocity+trim;
            end
            obj.plant.velocityControl(velocity,0);
            pause(0.5);
        end
    end
end