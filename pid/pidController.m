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
        function runPID(obj,targetState, margin)
            print('Running');
            output = [0,0,0];
	    % Checks for unreasonable input
            if margin == 0
                error('CANNOT HAVE ZERO ERROR.');
            end
            
	    % Running the currentStateFunc with a true argument is required to zero out the initial values of the encoders
            obj.currentState = obj.currentStateFunc(true);
            while ~obj.epsilonFunc('pid',targetState, margin)
                
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

%                 print('Current Error:\t%dmm\n',targetState - obj.currentState.val);    
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
        function runFF(obj,targetState, margin)
            print('Running');
            output = [0,0,0];
            
	    % Checks for unreasonable input
            if margin == 0
                margin('CANNOT HAVE ZERO ERROR.');
            end    
	    % Running the currentStateFunc with a true argument is required to zero out the initial values of the encoders
            vmax = 150;%mm
            amax = 200;%mm
            dist = targetState;%mm
            vFF = trapezoidalVelocityProfile(0,amax,vmax,dist);
            
            obj.currentState = obj.currentStateFunc(true);
            obj.plant.velocityControl(vFF,0);

            time = tic; % start time for integrator
            lastTime = toc(time);
            expectedDist = 0; %expected displacement
            while ~obj.epsilonFunc('pid',targetState, margin)

                currentTime = toc(time);
                dt = currentTime - lastTime;
                lastTime = currentTime;

                % current velocity value acording to our profile
                vFF = trapezoidalVelocityProfile(currentTime,amax,vmax,dist);
                error.val = expectedDist - obj.currentState.val;
                error.time= obj.currentState.time;

                % Proportional gain
                output(1) = error.val*obj.kp;

                % Integral Gain
                obj.integralError = obj.integralError + error.val;
                output(2) = obj.integralError*obj.ki;
                
                % Derivative Gain
                if obj.previousError.time == -1            
                    output(3) = 0;
                else
                    output(3) = ...
                    ((error.val - obj.previousError.val)/(error.time -obj.previousError.time))*obj.kd;
                end
                obj.previousError.val = error.val;
                obj.previousError.time = error.time;
                if obj.plotting
                    plotFeedforwardError(expectedDist,obj.currentState.val,error.val);
                end
                expectedDist = expectedDist + vFF * dt;
                fprintf('%d | %d | %d | %d || %d || %d\n',error.val,output(1),output(2),output(3),obj.integralError,dt);
                obj.pidOutFunc(sum(output),vFF);

                obj.currentState = obj.currentStateFunc(false);
                pause(0.05);
              
            end
            print('DONE');
            obj.plant.velocityControl(0,0);
            obj.integralError = 0;
            obj.previousError.time = -1;
        end
        
        function currentState = currentStateFunc(obj,initial)
            % Determines currentState
	    % These persistent values tare the currentState
            persistent encoderStart
                if initial 
                    encoderStart = obj.plant.currLeft + obj.plant.currRight;
                    
                    currentState.time = obj.plant.currTime;
                    currentState.x = 0;
                    currentState.y = 0;
                    currentState.theta = 0;
                end
                currentState.lastTime = obj.currentState.time;
                currentState.time = obj.plant.currentTime;
                currentState.lastS = obj.currentState.S;
                currentState.S = 0.5*(obj.plant.currLeft + obj.plant.currRight - encoderStart);
                currentState.v = obj.plant.velocity * 1000;
                currentState.omega = obj.plant.omega * 1000;
                currentState.theta = obj.currentState.theta + obj.currentState.omega * (obj.currentState.lastTime - obj.currentState.time);
                currentState.x = obj.currentState.x + obj.currentState.v * cos(obj.currentState.theta) * (obj.currentState.lastTime - obj.currentState.time);
                currentState.y = obj.currentState.y + obj.currentState.v * sin(obj.currentState.theta) * (obj.currentState.lastTime - obj.currentState.time);
        end
        
        function valid = epsilonFunc(obj,type, targetState, error)
            % Determines if currentState is within error of targetState
            if strcmp(type,'pid')
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
            elseif strcmp(type,'ff')
                if abs(obj.currentState.val - targetState)>error
                    valid = false;
                    return;
                end
                valid = true;
                return;
            else
                error('WRONG TYPE FOR EPSILONFUNC.');
            end
        end
        
        function pidOutFunc(obj, controlSig,velocity)
            % Outputs our control signal to the robot.       
            global max min
            vff = velocity;
            if controlSig > max 
                max = controlSig;
            end
            if controlSig < min
                min = controlSig;
            end
            
            trim = controlSig*0.001;
            if velocity+trim > 300
                velocity = 300;
            elseif velocity+trim < -300
                velocity = -300;
            else
                velocity = velocity+trim;
            end
            plotOutput(vff,velocity);
%             fprintf('%d | %d | %d\n',vff,velocity,trim);
%             fprintf('trim: %d \n VFF: %s \n  actuatedVel: %d \n xxxxxxxxxxxxxx',trim,vff,velocity);
            obj.plant.velocityControl(velocity/1000,0);
        end
    end
end