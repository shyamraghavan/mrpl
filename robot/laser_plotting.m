% rob = neato('sim');
% disp('Simulator launched.');
% 
%     % Standard Wall+Obstacle
%     walls = lineObject();
%                   % x  y
%     walls.lines = [-1 -1;
%                     2 -1;
%                     2  2; 
%                    -1  2; 
%                    -1 -1];
%     
%     ob1 = lineObject();
%     ob1.lines = [0.8 0.8; 1 0.8; 1 1; 0.8 1; 0.8 0.8];
%     
%     map = lineMap([walls ob1]);
%     rob.genMap(map.objects)
%     disp('Populated robot''s map.');

run('laserFigConfig.m');
laserFigUpdateListener = event.listener(rob.laser,'OnMessageReceived',@onNewLaserData);

% rob.sendVelocity(0.1,0.2);
% pause(2);
% rob.sendVelocity(0.3,0.2);
% pause(2);
% rob.sendVelocity(0.3,0.2);
% pause(2);
% rob.sendVelocity(0.3,0.2);
% pause(2);
% rob.sendVelocity(0.1,0.2);