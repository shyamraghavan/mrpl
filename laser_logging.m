rob = neato('sim');
disp('Simulator launched.');

    % Standard Wall+Obstacle
    walls = lineObject();
                  % x  y
    walls.lines = [-1 -1;
                    2 -1;
                    2  2; 
                   -1  2; 
                   -1 -1];
    
    ob1 = lineObject();
    ob1.lines = [0.8 0.8; 1 0.8; 1 1; 0.8 1; 0.8 0.8];
    
    map = lineMap([walls ob1]);
    rob.genMap(map.objects)
    disp('Populated robot''s map.');
    
rob.startLaser();
pause(0.2);
disp('Robot''s laser is activated.');
% 
% global logIndex laserLoggingListener;
% logIndex = 0;
% 
% delete('laserData.csv');
% laserLoggingListener = event.listener(rob.laser,'OnMessageReceived',@logLaserData);
% 
% rob.sendVelocity(0.1,0.2);s
% pause(2);
% rob.sendVelocity(0.3,0.2);
% pause(2);
% rob.sendVelocity(0.3,0.2);
% pause(2);
% rob.sendVelocity(0.3,0.2);
% pause(2);
% rob.sendVelocity(0.1,0.2);
% pause(4);
% 
% rob.shutdown();
% disp('Simulator closed.');
% close all;
% clear all;
