classdef pidController < handle
    properties
        % type of controler: PID, PD, PI,....
        type = 'pid';
         
        plotting = false;
        % Gains
        kp = 0;
        ki = 0;
        kd = 0;

        
        currentState = struct('time',-1,...
                              'lastTime',-1,...
                              'theta',-1,...
                              'x',-1,...
                              'y',-1,...
                              'v',-1,...
                              'omega',-1,...
                              'S',-1);
                              
        perceivedState = struct('val',-1,...
                              'time',-1);
        plant;
        
        % Variables required for the integral and differential error terms
        previousError = struct('val',-1,...
                               'time',-1);
        %proportional gain the first time around the loop?
        integralError=0;
        
        linVel=0;
        angVel=0;
        
        refControl;
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
        function runFF(obj, margin)
            print('Running');
            output = [0,0,0];
            
            obj.refControl = ReferenceControl();
            obj.refControl.figure8(0.5,0.5,1.5);
            
	    % Checks for unreasonable input
            if margin == 0
                margin('CANNOT HAVE ZERO ERROR.');
            end    
            
            vFF = [0 0];
            obj.currentState = obj.currentStateFunc(true);
            obj.plant.velocityControl(vFF(1),vFF(2));

            time = tic; % start time for integrator
            lastTime = toc(time);
            expected.x = -1;
            expected.y = -1;
            while ~obj.epsilonFunc('pid', expected, margin)

                currentTime = toc(time);
                dt = currentTime - lastTime;
                lastTime = currentTime;

                % current velocity value acording to our profile
                currentTime
                vFF = obj.refControl.computeControl(currentTime);
                if initial
                    expected.time = obj.plant.currTime;
                    expected.x = 0;
                    expected.y = 0;
                    expected.theta = 0;
                end
                expected.lastTime = expected.time;
                expected.time = obj.plant.currTime;
                expected.v = obj.linVel * 1000;
                expected.omega = obj.angVel * 1000;
                expected.theta = expected.theta + expected.omega * (expected.lastTime - expected.time);
                expected.x = expected.x + expected.v * cos(expected.theta) * (expected.lastTime - expected.time);
                expected.y = expected.y + expected.v * sin(expected.theta) * (expected.lastTime - expected.time);
                
                error.val = sqrt((expected.y - obj.currentState.y)^2 + (expected.x - obj.currentState.x)^2);
                
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
                expectedDist = expectedDist + vFF(1) * dt;
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
                currentState.time = obj.plant.currTime;
                currentState.v = obj.plant.velocity * 1000;
                currentState.omega = obj.plant.omega * 1000;
                currentState.S = 0.5*(obj.plant.currLeft + obj.plant.currRight - encoderStart);
                currentState.theta = obj.currentState.theta + obj.currentState.omega * (obj.currentState.lastTime - obj.currentState.time);
                currentState.x = obj.currentState.x + obj.currentState.v * cos(obj.currentState.theta) * (obj.currentState.lastTime - obj.currentState.time);
                currentState.y = obj.currentState.y + obj.currentState.v * sin(obj.currentState.theta) * (obj.currentState.lastTime - obj.currentState.time);
        end
        
        function valid = epsilonFunc(obj,type, expected, maxError)
            
            if expected.x == -1 && expected.y == -1
                valid = false;
                return
            end
            % Determines if currentState is within error of targetState
            
            if strcmp(type,'pid')
            % if we are not within boundaries we short circuit the evaluation.
                currError = sqrt((expected.y - obj.currentState.y)^2 + (expected.x - obj.currentState.x)^2);
                if currError > maxError 
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
            else
                error('WRONG TYPE FOR EPSILONFUNC.');
            end
        end
        
        function pidOutFunc(obj, controlSig,velocity)
            % Outputs our control signal to the robot.       
            global max min
            linVel = velocity(1);
            angVel = velocity(2);
            vff = velocity;
            if controlSig > max 
                max = controlSig;
            end
            if controlSig < min
                min = controlSig;
            end
            
            trim = controlSig*0.001;
            if linVel+trim > 300
                linVel = 300;
            elseif linVel+trim < -300
                linVel = -300;
            else
                linVel = linVel+trim;
            end
            
            angVelMax = (300 - linVel)*2/0.2350;
            
            if angVel+trim > angVelMax
                angVel = 300;
            elseif angVel+trim < -angVelMax
                angVel = -300;
            else
                angVel = angVel+trim;
            end
            plotOutput(vff,linVel);
%             fprintf('%d | %d | %d\n',vff,velocity,trim);
%             fprintf('trim: %d \n VFF: %s \n  actuatedVel: %d \n xxxxxxxxxxxxxx',trim,vff,velocity);
            obj.plant.velocityControl(linVel/1000,angVel/1000);
            
            obj.linVel = linVel;
            obj.angVel = angVel;
        end
    end
end