function testFeedForward
    clear all;close all;clc;

    vmax = 0.25;
    amax = 3*0.25;
    dist = -1;

    r = robot('centi');
    figure
    hold on;
    pause(1);
    initialLocation = r.encoders.left;

    u = trapezoidalVelocityProfile(0,amax,vmax,dist);
    r.velocityControl(u,0);
    scatter(0,u);
    distanceIntegral = 0;
    count = 0;
    tic
    while toc < abs(dist)*4
        u = trapezoidalVelocityProfile(toc,amax,vmax,dist);
        distanceIntegral = distanceIntegral + u;
        count = count + 1;
        r.velocityControl(u,0);
        scatter(toc,u,'+r');
    end
    disp(distanceIntegral/count);
    r.velocityControl(0,0);

    fprintf('Wheel movement: %dmm\n',r.encoders.left - initialLocation);
end
