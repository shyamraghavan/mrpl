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
        % afterwards. Kv scales velocity: v = Vmax*Kv and Ks scales
        % the angular velocity: w=wMax*Ks. 
        
            if (abs(Ks)>1) || (abs(Kv)>1)
                error('Invalid Inputs.');
            end
            
            obj.type = 'figure8';
            tMax = 4*pi/(Kv*Ks);
            
            % calculates the trajectory
            obj.t = 0:0.1:tMax;
            obj.v = 0.3*Kv.*ones(1,length(obj.t));
            obj.w = (Kv/Ks).*sin((obj.t.*Kv)./2*Ks)/(1.2*pi);
            
            %zeroes out the velocities during the leading and tailing
            %pauses.
            obj.t = [0 tPause-0.001 obj.t+tPause obj.t(end)+tPause-0.001 tMax+2*tPause];
            obj.v = [0 0 obj.v 0 0];
            obj.w = [0 0 obj.w 0 0];
        end

        function circle(obj,radius,Kv,tPause)
            if (abs(Kv)>1)||(radius <= 0)
                error('Invalid Inputs')
            end
            
            obj.type = 'circle';
            tMax = (2*pi*radius)/(Kv*0.3);
            
            k = 1/radius;
            tTrajectory = 0:0.1:tMax;
            obj.t = [0 tPause-0.001 tTrajectory+tPause tTrajectory(end)+tPause-0.001 tMax+2*tPause];
            obj.v = [0 0 Kv*0.3.*ones(1,length(tTrajectory)) 0 0];
            obj.w = obj.v./k;
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
        
        function type = getType(obj)
            type = obj.type;
        end
    end
end