function populateSimMap(obj,n)
% Loads map number n into the simulator.
n;
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
obj.rob.genMap(map.objects)
disp('Populated robot''s map.');
    
end

