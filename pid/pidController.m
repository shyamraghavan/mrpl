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
%                 print('start')
		% Proportional gain
                output(1) = (targetState - obj.currentState.val)*obj.kp;
%                 print(output(1));
              	
		% Integral Gain
                output(2) = ((targetState - obj.currentState.val)...
                            + obj.integralError)*obj.ki;
                obj.integralError = obj.integralError + (targetState - obj.currentState.val);
                
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
%                 print(output(3));
            fprintf('Current Error:\t%dmm\n',targetState - obj.currentState.val);    
            plotDistanceError(obj.currentState, targetState);
            disp(output);
        	obj.pidOutFunc(sum(output))
           	obj.currentState = obj.currentStateFunc(false);

            end 
            print('DONE');
            obj.plant.velocityControl(0,0);
            obj.integralError = 0;
            obj.previousError.time = -1;
            
        end
        
        %% Run with Feed Forward
        function runFF(obj,targetState, error)
            print('Running');
            output = [0,0,0];            
    
	    % Running the currentStateFunc with a true argument is required to zero out the initial values of the encoders
            obj.currentState = obj.currentStateFunc(true);
            
            
	    % Checks for unreasonable input
            if error == 0
                error('CANNOT HAVE ZERO ERROR.');
            end
            
            if 1
                vmax = 0.25;
                amax = 3*0.25;
                dist = targetState/1000;

                initialLocation = obj.plant.encoders.left;
                
                figure(3);
                errorPlot = subplot(2,1,1);
                plot(errorPlot,1:10,1:10,'+m');
                title(errorPlot,'Distance Error (m)');
                set(errorPlot,'Tag','feedForwardErrorPlot',...
                              'YMinorGrid', 'on');
                ffPlot = subplot(2,1,2);
                plot(ffPlot,1:10,1:10,'+r');
                hold on;
                plot(ffPlot,1:10,1:10,'+g');
                hold off;
                title(ffPlot,'Distance Error (m)');
                set(ffPlot,'Tag','feedForwardPlot',...
                              'YMinorGrid', 'on');
                
                
                
                u = trapezoidalVelocityProfile(0,amax,vmax,dist);
                obj.plant.velocityControl(u,0);
               
                tic
                time = toc;
                distanceIntegral = 0;
                while toc < abs(dist)*4
                    lastTime = time;
                    time = toc;
                    dt = time - lastTime;
                    u = trapezoidalVelocityProfile(toc,amax,vmax,dist);
                    obj.plant.velocityControl(u,0);
                    distanceIntegral = distanceIntegral + u * dt;
                    
                    obj.currentState = obj.currentStateFunc(false);
                    plotFeedforwardError(distanceIntegral*1000,(obj.plant.encoders.left-initialLocation))
                    pause(0.01);
                end

                obj.plant.velocityControl(0,0);

                fprintf('Wheel movement Feed Fwd: %dmm\n',obj.plant.encoders.left - initialLocation);
                
%                 targetState = obj.plant.encoders.left - initialLocation - targetState;
                fprintf('New Target State: %dmm\n',targetState);
            end
            
            
            while ~obj.epsilonFunc(targetState, error)
%                 print('start')
		% Proportional gain
                output(1) = (targetState - obj.currentState.val)*obj.kp;
%                 print(output(1));
              	
		% Integral Gain
                output(2) = ((targetState - obj.currentState.val)...
                            + obj.integralError)*obj.ki;
                obj.integralError = obj.integralError + (targetState - obj.currentState.val);
                
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
%                 print(output(3));
%             fprintf('Current Error:\t%dmm\n',targetState - obj.currentState.val);    
%             disp(obj.currentState);
            plotDistanceError(obj.currentState, targetState);
        	obj.pidOutFunc(sum(output))
           	obj.currentState = obj.currentStateFunc(false);

            end 
            print('DONE');
            obj.plant.velocityControl(0,0);
            obj.integralError = 0;
            obj.previousError.time = -1;
            global data
            hold on;
            plot(ffPlot,1:size(data,2),data(1,:),'+r');
            plot(ffPlot,1:size(data,2),data(2,:),'+g');
            hold off;
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
            velocity = controlSig*0.001;
            if velocity > 0.3
                velocity = 0.3;
            elseif velocity <-0.3
                velocity = -0.3;
            end
            global error;
            obj.plant.velocityControl(velocity,0);
            pause(0.5);
        end
    end
end