classdef robot < handle

     properties
        SIM_STR = 'sim';
        useSim = false;
        neatoRobot;
        robotActive = false;
        robotMapped = false;
        deadReckoning = struct('xPos',0,...
                               'yPos',0,...
                               'thPos',0)
        lidarData = zeros(1,360);
        encoders = struct('leftPast',-1,...
                          'rightPast',-1,...
                          'timePastSec',-1,...
                          'timePastNSec',-1);                   
        mapNumber = 0;
        laserFigUpdateListener = -1;
        laserLoggingListener = -1;
        estimator = -1;
        encoderListener = -1;
        wheelbase = 0.2350;
        
        % Previous Timestamp
        prevTimeStamp = -1;
        % Current Timestamp
        timeStamp = -1;
        % Delta T
        dt = -1
        leftWheelPos = -1;
        prevLeftWheelPos = -1;
        rightWheelPos = -1;
        prevRightWheelPos = -1;
        % Change in left wheel position.
        leftdS = 0;
        % Change in right wheel position.
        rightdS = 0;
        leftV = 0;
        rightV = 0;
        % Angle of robots movement
        posePlot;
        xPositions = zeros(1,1000);
        yPositions = zeros(1,1000);
        loopIndex = 1;
        omega = 0;
        theta = 0;
        v = 0;
     end

    methods 

        function obj = robot(robotName,varargin)
            % Init neato robot.
            obj.neatoRobot = neato(robotName);
            obj.robotActive = true;
            
            % Check if rob is a robot or a simulator
            if strcmp(robotName,obj.SIM_STR)
                disp('Created Simulator.');
                obj.useSim = true;
            else
                disp('Connected to Robot.');
                obj.useSim = false;
            end
            
            
%             obj.estimator = event.listener(obj.neatoRobot.laser,...
%                  'OnMessageReceived',@(src,evt) estimator(src,evt,obj)); 

             obj.encoderListener = event.listener(obj.neatoRobot.encoders,...
                 'OnMessageReceived',@(src,evt) encoderListener(src,evt,obj));
            % Evaluate optional arguments passed to the robot.
            skip = false;
            index = -1;
            
            for k = 1:length(varargin)
                % Handles skipping over data fields.
                if skip && (k == index)
                    index = -1;
                    skip = false;
                    continue
                end
                
                % Parses the different optional arguments.
                switch varargin{k}
                    case 'Laser'
                        % Starts the laser plot and its listener.
                        obj.setLaserFig(true);
                        
                    case 'LaserLog'
                        % Starts logging the laser data to a csv file.
                        delete('laserData.csv');
                        obj.setLaserLogging
                                        
                    case 'Map'
                        % Define which map to load into the simulator.
                        if obj.useSim
                            obj.mapNumber = varargin{k+1};
                            populateSimMap(obj,obj.mapNumber);
                        else
                            warning('Attempted to define map for non-sim robot');
                        end
                        skip = true;
                        index = k+1;
                                            
                    case 'Pose'
                        % Define star values for xPos,yPos, and thPos.
                        poseData = varargin{k+1};
                        setPos(poseData(1),poseData(2),poseData(3));
                        skip = true;
                        index = k+1;
                        
                    otherwise
                        % Warns user of unrecognized arguments.
                        warning(['Argument #',num2str(k+1),': ',varargin{k},...
                            ' was not recognized.']);
                end
            end
        end
        
        function setLaserFig(obj,value)
            if value
                disp('LIDAR started.');
                
                if obj.useSim
                    if ~obj.robotMapped
                        populateSimMap(obj,obj.mapNumber);
                    end
                    
                    obj.neatoRobot.startLaser()
                    pause(0.5)              
                else
                    obj.neatoRobot.startLaser()
                    pause(4);
                end
                
                disp('LIDAR operational.');
                
                run('laserFigConfig.m');
                obj.laserFigUpdateListener = event.listener(obj.neatoRobot.laser,...
                 'OnMessageReceived',@onNewLaserData);
            else
                if obj.laserFigUpdateListener ~= -1
                    delete(obj.laserFigUpdateListener);
                end
                  
                if obj.neatoRobot.laser.isvalid()
                    obj.neatoRobot.stopLaser()
                end
                
                close(findobj('Tag','laserFig'));
                disp('LIDAR stopped.');
            end
        end
        
        function setLaserLogging(obj,value)
            if value
                disp('Data logging started');
                obj.laserLoggingListener = event.listener(obj.neatoRobot.laser,...
                    'OnMessageReceived',@logLaserData);
            else
                if obj.laserLoggingListener ~= -1
                    delete(obj.laserLoggingListener);
                end
                disp('Data logging stopped.');
            end
        end
        
        function velocityControl(obj, v, omega)
            vr = v + obj.wheelbase / 2 * omega;
            vl = v - obj.wheelbase / 2 * omega;
            obj.neatoRobot.sendVelocity(vr, vl);
        end
        
        function goToXY(obj, deltaX, deltaY, deltatheta, velocity)
            kappa = deltatheta / sqrt(deltaX^2 + deltaY^2);
            omega = kappa * velocity;
            obj.velocityControl(velocity, omega);
        end
        
        function setPos(obj,x,y,theta)
            obj.xPos = x;
            obj.yPos = y;
            obj.thPos = theta;
        end
        
        function [x,y,theta] = getPos(obj)
            x = obj.deadReckoning.xPos;
            y = obj.deadReckoning.yPos;
            theta = obj.deadReckoning.thPos;
        end
        
        function close(obj)
            obj.setLaserFig(false);
            obj.setLaserLogging(false);
            
            % Close robots, shutdown simulators
            if obj.useSim
                obj.neatoRobot.shutdown();
                disp('Simulator shut down.');
            else
                obj.neatoRobot.close()
                disp('Robot Connection Closed.');
            end
            obj.robotActive = false;
        end
        
        function delete(obj)
            if obj.robotActive
                close(obj);
            end
        end
    end
end