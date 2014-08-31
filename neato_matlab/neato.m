classdef neato < handle
    
    properties (GetAccess='public', SetAccess='private')
		name = 'sim';
		ws;
		vel_pub;
		laser_pub;
		fork_pub;		
		kill_pub;
		encoders;
		%bumper;
		laser;
		battery;
		sim_robot;
		map;
    end
    
    properties (Hidden, GetAccess='private', SetAccess='private')
		sim = true;
		update_timer;
		stamper;
		last_update;

		ph = [-1 -1]; %plot handle
		ah;
		dist_since_cmd = 0;
		last_x = 0;
		last_y = 0;
		last_th = 0;
		
		enc_cnt = 0;
		laser_cnt = 0;
		plot_cnt = 5;
		last_l = 0;
		last_r = 0;
		vel_cmd = [0 0];
		
		laser_beam_pos = 0;
		sim_laser_on = false;
		has_map = false;
        
        fifoMutex = 0;
        tLog
        logDirName
    end
    
    properties (Constant, Hidden, GetAccess='private')
		max_vel = .3;
        default_gateway = '192.168.0.1';
		
		sim_freq = 90;				
		enc_prescaler = 3; % 30 Hz enc updates
		
        laser_prescaler = 22; % ~4 Hz raycasting, full sweep
		laser_sweep = 360;
		laser_range = 4.5;
		
		plot_prescaler = 5; %90/5 = 18 Hz
				
		enc_delay = .125; %enc and vel delay
		laser_delay = .4; 
		max_dist = .5;
		
		wheel_base = .248;
		rad = .165;
		laser_l = -0.100;
		laser_rad = 0.04;
        
        logFileName = 'neato_usage_log.txt';
    end
    
    methods
		function r = neato(name,pose)
            if(nargin > 0)
				r.name = name;
            end
            if ~isequal(r.name,'sim')
                r.verifyNetwork();
            end
            
            % Addpath to files.
            fname = which('neato.m');
            fname = strrep(fname,'neato.m','');
            addpath(genpath(fname));
            
            if(nargin > 1)
                if( all( size(pose) == [3 1] ) )
                    initialPose = pose;
                else
                    error('Initial pose must be 3 x 1');
                end
            else
                initialPose = [0;0;0];
            end
            if(strcmp(name,'sim') || strcmp(name,'manual_sim') )
                r.sim_robot =  simRobot(r.enc_delay, initialPose, 0, 0, false);
				r.sim_robot.fireUpForRealTime();
				
				r.stamper = tic();
				r.last_update = tic();
				
				r.encoders = SimSubscriber(struct('left',0,'right',0));
				r.laser = SimSubscriber(struct('ranges',zeros(1,360)));
				
				if(~strcmp(name,'manual_sim'))
					r.update_timer = timer;
                    r.update_timer.Tag = 'neato_sim';
					r.update_timer.ExecutionMode = 'fixedRate';
					r.update_timer.BusyMode = 'queue';
                    %r.update_timer.BusyMode = 'drop';
					r.update_timer.TimerFcn = @r.simUpdate;
                    s = warning('off', 'MATLAB:TIMER:RATEPRECISION');
					r.update_timer.Period = 1/r.sim_freq;
                    warning(s);
				
					start(r.update_timer);
				end
            else
                r.sim = 0;
				try
					r.ws =  ros_websocket(['ws://' r.default_gateway ':9090']);
				catch e
					error(['Cant open socket, first try to "clear all", then try again'...
						'\n If that does not work, the robot is off, you are not '...
						'connected to MRP2, or the robot needs to be restarted'])
				end
				%robot.ws =  ros_websocket('ws://hecto:9090');
				r.vel_pub =  Publisher(r.ws,['/vel'],'neato_cpp/TwoInts');
				r.laser_pub =  Publisher(r.ws,['/laser'],'std_msgs/Int8');
				r.fork_pub =  Publisher(r.ws,['/forks'],'std_msgs/UInt8');
				r.kill_pub =  Publisher(r.ws,['/kill'],'std_msgs/Int8');
				r.encoders = Subscriber(r.ws,['/enc'],'neato_cpp/TwoInts');
				%r.bumper = Subscriber(r.ws,['/' name '/bump'],'neato_cpp/FourInts');
				r.laser = Subscriber(r.ws, ['/scan'], 'sensor_msgs/LaserScan');
			end
      %{
			r.encoders.data.left = 0;
			r.encoders.data.right = 0;
			r.encoders.data.header.stamp = r.timestamp();
      %}
            str = which('neato.m');
            r.logDirName = strrep(str,'neato.m','usage_log');
            if ~exist(r.logDirName,'dir')
                mkdir(r.logDirName);
            end
            r.tLog = tic();
        end
		
        function res = timerUp(r)
            if r.sim
                res = timerfind('Tag','neato_sim');
                res = ~isempty(res);
            else
                fprintf('Robot not in simulation mode.\n');
                res = 0;
            end
        end
        
        function shutdown(r)
            fname = sprintf('%s/%s',r.logDirName,neato.logFileName);
            if ~exist(fname,'file')
                fid = fopen(fname,'w');
            else
                fid = fopen(fname,'a');
            end
            vec = fix(clock);
            fprintf(fid,'%d-%d-%d-%d-%d %s %f\n',vec(1),vec(2),vec(3),vec(4),vec(5),r.name,toc(r.tLog));
            fclose(fid);
            
            if r.sim
                stop(r.update_timer);
                delete(r.update_timer);
			else
				try
					msg = struct();
					msg.data = 1;
					r.kill_pub.publish(msg);
					r.kill_pub.publish(msg);
					r.ws.delete;
				catch e
					error('Tried to shutdown robot, but not connected');
				end
            end
        end
		
		function r = close(r)
			if(~r.sim)
				r.ws.delete;
			end
		end		
		
		function forksUp(r)
			if r.sim
				warning('No forks in simulation... yet');
			else
				if r.ws.isvalid
					msg = struct();
					msg.data = 180;
					r.fork_pub.publish(msg);
				else
					error('Robot Connection is not valid')
				end
			end
		end		
		
		function forksDown(r)
			if r.sim
				warning('No forks in simulation... yet');
			else
				if r.ws.isvalid
					msg = struct();
					msg.data = 0;
					r.fork_pub.publish(msg);
				else
					error('Robot Connection is not valid')
				end
			end
		end	
		
		function startLaser(r)
			if r.sim
				r.sim_laser_on = true;
                if ~r.has_map
                    warning('No map, are you sure you want to run the simulated laser?');
                end
			else
				if r.ws.isvalid
					msg = struct();
					msg.data = 1;
					r.laser_pub.publish(msg);
				else
					error('Robot Connection is not valid')
				end
			end
		end

		function stopLaser(r)
			if r.sim
				r.sim_laser_on = false;
			else
				if r.ws.isvalid
					msg = struct();
					msg.data = 0;
					r.laser_pub.publish(msg);
				else
					error('Robot Connection is not valid')
				end
			end
		end
		
		function sendVelocity(r, v_l, v_r)
			if( abs(v_l) > r.max_vel || abs(v_r) > r.max_vel)
				error(['Max Vel is ' num2str(r.max_vel) ...
					', you sent ' num2str(max(abs([v_l,v_r])))]);
			end
			
			if(r.sim)
                r.fifoMutex = 1;
				r.sim_robot.sendVelocity(v_l, v_r);
                r.fifoMutex = 0;
				r.dist_since_cmd = 0;
			else
				msg = struct();
				msg.left = round(v_l*1000);
				msg.right = round(v_r*1000);
				if msg.right == 0 && msg.left == 0
					msg.right = -1;
					msg.left = -1;
				end
				r.vel_pub.publish(msg);
			end
			r.vel_cmd = [v_l v_r];
		end
		
		function manual_update(r)
			toc(r.last_update)
			if( toc(r.last_update) > 1/r.sim_freq)
				r.simUpdate(0,0);
			end
		end		
		
		function genMap(r, obs)
			fig_num = get(r.ah,'Parent');
			r.map = lineMap(obs);
			r.has_map = true;
        end		
    
        function togglePlot(r)
            if ishandle(r.ph(1))
                hfig = get(get(r.ph(1),'parent'),'parent');
                hState = get(hfig,'visible');
                switch hState
                    case 'on'
                        set(hfig,'visible','off');
                    case 'off'
                        set(hfig,'visible','on');
                end
            end
        end
        
        function verifyNetwork(obj)
            if ispc
                obj.verifyNetworkWindows();
            elseif ismac
                obj.verifyNetworkMac();
            else
                obj.verifyNetworkLinux();
            end
        end
        
        function verifyNetworkWindows(obj)
            [status,response] = system('netsh wlan show interfaces');
            parsed = strsplit(response);
            flag = cellfun(@(x) isequal(x,'State'),parsed);
            id = find(flag); id = id+2;
            if ~isequal(parsed{id},'connected')
                error('NOT CONNECTED TO A NETWORK.');
            end
            flag = cellfun(@(x) isequal(x,'SSID'),parsed);
            id = find(flag); id = id+2;
            if isequal(parsed{id},'MRP2') || isequal(parsed{id},obj.name)
                return
            else
                error('NEED TO BE CONNECTED TO NETWORK %s OR MRP2.',upper(obj.name));
            end
        end
        
        function verifyNetworkMac(obj)
            [status,response] = system('/System/Library/PrivateFrameworks/Apple80211.framework/Versions/Current/Resources/airport -I');
            if strfind(response,'AirPort')
                 error('NOT CONNECTED TO A NETWORK.');
            end
            parsed = strsplit(response);
            flag = cellfun(@(x) isequal(x,'SSID:'),parsed);
            id = find(flag); id = id+1;
            if isequal(parsed{id},'MRP2') || isequal(parsed{id},obj.name)
                return
            else
                error('NEED TO BE CONNECTED TO NETWORK %s OR MRP2.',upper(obj.name));
            end
        end
        
        function verifyNetworkLinux(obj)
            % TODO: debug
            [status,response] = system('iwgetid -r');
            response = strtrim(response);
            if isempty(response)
                error('NOT CONNECTED TO A NETWORK.');
            end
            if isequal(response,'MRP2') || isequal(response,obj.name)
                return
            else
                error('NEED TO BE CONNECTED TO NETWORK %s OR MRP2.',upper(obj.name));
            end
        end
    end
	
	methods (Hidden = true, Access = 'private')
        function simUpdate(r,caller,event)
            if ~r.fifoMutex
                r.sim_robot.sendVelocity(r.vel_cmd(1), r.vel_cmd(2));
            end
			r.sim_robot.updateState();
			
			el = (r.sim_robot.encoders.data.left - r.last_l)/1000;
			er = (r.sim_robot.encoders.data.right - r.last_r)/1000;
			r.last_l = r.sim_robot.encoders.data.left;
			r.last_r = r.sim_robot.encoders.data.right;
			d = max(abs(el),abs(er));
			r.dist_since_cmd = r.dist_since_cmd + d;
			
			%stop the robot if it hasn't been commanded and has gone
			%max_dist
			if(r.dist_since_cmd > r.max_dist)
					r.vel_cmd = [0 0];
			end

			%Every third update, update the encoders
			if(r.enc_cnt == r.enc_prescaler)
				r.encoders.data = r.sim_robot.encoders.data;
				r.encoders.data.header.stamp = r.timestamp();
				r.encoders.publish();
				r.enc_cnt = 0;
			end
			
			if(r.laser_cnt == r.laser_prescaler)
				if r.sim_laser_on
                    if r.has_map
                        l_pose = r.sim_robot.pose;
                        l_pose(1:2) = l_pose(1:2) + ...
                            r.laser_l.*[cos(l_pose(3)); sin(l_pose(3))];
                        
                        [r.laser.data.ranges,~] = ...
                            r.map.raycast(l_pose,r.laser_range,deg2rad(0:359));
                    else
                        r.laser.data.ranges = zeros(360,1);
                    end
                end
                r.laser.data.header.stamp = r.timestamp();
                r.laser.publish();
				r.laser_cnt = 0;
			end
			
			if(r.plot_cnt == r.plot_prescaler)
				r.plot();
				drawnow;
				r.plot_cnt = 0;
			end
			
			r.enc_cnt   = r.enc_cnt+1;
			r.laser_cnt = r.laser_cnt+1;
			r.plot_cnt  = r.plot_cnt+1;
			
			%start the timer for to check timing
			r.last_update = tic();
		end	
		
		function stamp = timestamp(r)
			t = toc(r.stamper);
			stamp.secs = floor(t);
			stamp.nsecs = (t - floor(t)) * 1000000000;
		end

		function plot(r)
			step = pi/20;
			q1 = 0:step:pi/2;
			q2 = pi/2:step:pi;
			cir = 0:step:2*pi;
			
			lx = (r.laser_rad*-cos(cir)) + r.laser_l;
			ly = r.laser_rad*sin(cir);
			
			bx = [-sin(q1)*r.rad lx [-sin(q2) 1  1  0]*r.rad];
			by = [-cos(q1)*r.rad ly [-cos(q2) 1 -1 -1]*r.rad];
			
			x = r.sim_robot.pose(1);
			y = r.sim_robot.pose(2);
			theta = r.sim_robot.pose(3);
			
			update_plot = true;
			if( x == r.last_x && y == r.last_y && theta == r.last_th)
				update_plot = false;
			end
			r.last_x = x;
			r.last_y = y;
			r.last_th = theta;
			
			px = x + (cos(theta)*bx + sin(theta)*by);
			py = y + (sin(theta)*bx - cos(theta)*by);
			if ishandle(r.ph(1))
				if(update_plot)
					set(r.ph(1),'XData',px,'YData',py);
					set(r.ph(2),'XData',[get(r.ph(2),'XData') x],...
						'YData',[get(r.ph(2),'YData') y]);
				end
			else
				figure();
				r.ah = gca();
				r.ph(1) = plot(px,py,'-','LineWidth',2,'Color',[.02 .3 .05]);
				hold on;
				r.ph(2) = plot(x,y,'-','LineWidth',1,'Color',[.4 .4 .9]);
				hold off;
				%axis([x+[-1 1] y+[-1 1]]);
				axis equal
				grid on
			end
			
			if r.has_map
				r.map.update;
			end
        end
        
        function delete(r)
            % Destructor
        end
    end
end
