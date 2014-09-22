classdef pidController < handle
    properties
        % type of controler: PID, PD, PI,....
        type = 'xxx';
        
        % Gains
        kp = 0;
        ki = 0;
        kd = 0;
       
        % Function handles for the different components of the controllers
        pErrorFunc;
        iErrorFunc;
        dErrorFunc;
        
        plant;
        previousState;
        maxSig = 1;  

        % Function handles required for the operation of the controller
        epsilonFunc; % returns true if we are within acceptable margins
        currentStateFunc; % updates the current state
        pidOutFunc; % applies our output signal to the plant
        funcVec = [0 0 0]; % vector used for validity testing.
        
        % Variables required for the integral and differential error terms
        previousError=0; %Should this be zero? or should we ignore the 
        %proportional gain the first time around the loop?
        integralError=0;
    end
    methods
        %% Constructor
        function obj = pidController(plantObj)
            obj.plant = plantObj;
        end
        
        %% Setters for the different types of control. 
         % Each requires their own error function handles.
        function setPErrorFunc(obj,errorFunc)
            % of the form: errorFunc(k, currentState, targetState)
            obj.pErrorFunc=errorFunc;
            obj.type(1) = 'p';
        end
        
        function setIErrorFunc(obj,errorFunc)
            % of the form: errorFunc(k, currentState, targetState, integralError)
            obj.iErrorFunc=errorFunc;
            obj.type(2) = 'i';
        end
        
        function setDErrorFunc(obj,errorFunc)
            % of the form: errorFunc(k, currentState, targetState, previousErrror)
            obj.dErrorFunc=errorFunc;
            obj.type(3) = 'd';
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
        
        
        %% Setters for the mandatory function handles
        function setEpsilonFunc(obj,epsilonFunc)
            % of the form: epsilon(currentState, targetState, error)
            obj.epsilonFunc=epsilonFunc;
            obj.funcVec(1) = 1;
        end
        
        function setCurrentStateFunc(obj,func)
            % of the form: currentState(plant)
            obj.currentStateFunc = func;
            obj.funcVec(2) = 1;
        end
        
        function setPidOutputFunc(obj,func)
            % of the form: pidOut(plant, output)
            obj.pidOutFunc = func;
            obj.funcVec(3) = 1;
        end
        
        
        %% Run is the main control loop of the controller
        function run(obj,targetState, error)
            obj.isValidController();
            output = [0,0,0];
            
            currentState = obj.currentStateFunc(obj.plant);
            while ~obj.epsilonFunc(currentState,targetState, error)
               obj.previousState = currentState;
               currentState = obj.currentStateFunc(obj.plant);
               
               if obj.type(1) == 'p'
                   output(1) = obj.pErrorFunc(obj.kp,...
                                                  currentState,...
                                                  targetState);
               end 
               if obj.type(2) == 'i'
                   output(2) = obj.dErrorFunc(obj.ki,...
                                                  currentState,...
                                                  targetState,...
                                              obj.integralError);
               end
               if obj.type(3) == 'd'
                   obj.previousError = obj.iErrorFunc(obj.kd,...
                                                          currentState,...
                                                         targetState,...
                                                      obj.previousError);
                  output(3) = obj.previousError.val;
               end
               obj.pidOutFunc(obj,cumsum(output))
               
            end          
            obj.integralError = 0;
            
        end
        
        
        %% Determines the integrety of the controller
        function valid = isValidController(obj)
            valid = 0;
            if (obj.type(1) == 'p') || ...
               (obj.type(2) == 'i') || ...
               (obj.type(3) == 'd')

                if isempty(find(obj.funcVec==0,3,'First'))
                    valid = 1;
                    return
                end
            end 
        end
        
    end
end