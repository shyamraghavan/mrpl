classdef ReferenceControl < handle
    properties
        Ks;
        Kv;
        tPause;
        
        t;
        v;
        w;
        
        type;
   end
    
    methods
        function obj = ReferenceControl()
            obj.type = 'none';
        end
        
        function figure8(obj,Ks,Kv,tPause)
        % Construct a figure 8 trajectory. It will not start until
        % tPause has elapsed and it will stay at zero for tPause
        % afterwards. Kv scales velocity up when > 1 and Ks scales
        % the size of the curve itself down. 0.5 is a good value 
        % for both. 
            obj.type = 'figure8';
            tMax = 4*pi/(Kv*Ks);
            
            % calculates the trajectory
            obj.t = 0:0.1:tMax;
            obj.v = 0.3*Kv.*ones(1,length(obj.t));
            obj.w = (Kv/Ks).*sin((obj.t.*Kv)./2*Ks);
            
            %zeroes out the velocities during the leading and tailing
            %pauses.
            obj.t = [0 tPause obj.t+tPause obj.t(end)+tPause tMax+2*tPause];
            obj.v = [0 0 obj.v 0 0];
            obj.w = [0 0 obj.w 0 0];
        end

        function [V,w] = computeControl(obj,timeNow)
        % Return the linear and angular velocity that the robot 
        % should be executing at time timeNow. Any zero velocity 
        % pauses specified in the constructor are implemented here
        % too.

            V = interp1(obj.t,obj.v,timeNow);
            w = interp1(obj.t,obj.w,timeNow);
        end

        function duration = getTrajectoryDuration(obj)
        % Return the total time required for motion and for the 
        % initial and terminal pauses.
            duration = obj.t(end);
        end
    end
end