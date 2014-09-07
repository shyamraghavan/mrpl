rob = neato('kilo');

v = 0.15;
differential = 0.08;

pause(1);
rob.startLaser();
pause(1);
disp('Robot''s laser is activated.');

run('laserFigConfig.m');
laserFigUpdateListener = event.listener(rob.laser,'OnMessageReceived',@onNewLaserData);

cleanedLaserRanges = rangeDataFilter(rob.laser.data.ranges);
[i,r] = nearestObject(inSector(cleanedLaserRanges, -45, 45));

while true
    [r,i]
    if  i == -1 || r > 1.75
        rob.sendVelocity(0,0);
    elseif r > 0.70
        if i < 45 & i > 10
            rob.sendVelocity(v - differential, v + differential);
        elseif i > 315 & i < 350
            rob.sendVelocity(v + differential, v - differential);
        else
            rob.sendVelocity(v,v);
        end
    elseif r < 0.60
        rob.sendVelocity(-v,-v);
    else
        rob.sendVelocity(0,0);
    end
    
    cleanedLaserRanges = rangeDataFilter(rob.laser.data.ranges);
    [i,r] = nearestObject(inSector(cleanedLaserRanges, -45, 45));
    
    pause(0.05);
end