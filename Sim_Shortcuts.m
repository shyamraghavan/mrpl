%% Simulator launch and shutdown script

%% Sim Shutdown
rob.shutdown();
close 1;
disp('Simulator closed.');

%% Sim Start
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