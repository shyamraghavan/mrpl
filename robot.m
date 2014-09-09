classdef robot < handle

     properties
        USE_SIM = false;
        rob = -1;
        simStr = 'sim';
     end

    methods 

        function obj = robot(robName)
            % Init neato robot.
            obj.rob = neato(robName);
            % Check if rob is a robot or a simulator
            if strcmp(robName,obj.simStr)
                disp('Created Simulator.');
                obj.USE_SIM = true;
            else
                disp('Connected to Robot.');
                obj.USE_SIM = false;
            end
            
        end

        function close(obj)
            obj.rob.stopLaser();
            % Close robots, shutdown simulators
            if obj.USE_SIM
                obj.rob.shutdown();
                disp('Simulator shut down.');
            else
                obj.rob.close()
                disp('Robot Connection Closed.');
            end
        end
    end    
end