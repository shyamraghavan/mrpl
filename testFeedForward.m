clc
function testFeedForward
    clear all;close all;clc;

    vmax = 0.25;
    amax = 3*0.25;
    dist = 1;

    r = robot('sim');
%     figure
    hold on;
    pause(2);
    initialLocation = r.encoders.left;

    u = trapezoidalVelocityProfile(0,amax,vmax,dist);
    r.velocityControl(u,0);
%     scatter(0,u);
    distanceIntegral = 0;
    count = 0;
    
    figure(3);
    errorPlot = subplot(2,1,1);
    plot(errorPlot,1:10,1:10,'+m');
    title(errorPlot,'Distance Error (m)');
    set(errorPlot,'Tag','feedForwardErrorPlot',...
                  'YMinorGrid', 'on');
         
    ffPlot = subplot(2,1,2);
    plot(ffPlot,1:10,1:10,'+r');
    hold on;
    plot(ffPlot,1:10,1:10,'+g');
    hold off;
    title(ffPlot,'Distance Error (m)');
    set(ffPlot,'Tag','feedForwardPlot',...
                  'YMinorGrid', 'on');
    
    
    tic
    time = toc;
    while toc < abs(dist)*4
        lastTime = time;
        time = toc;
        dt = time - lastTime;
        u = trapezoidalVelocityProfile(toc,amax,vmax,dist);
        distanceIntegral = distanceIntegral + u * dt;
        
        count = count + 1;
        r.velocityControl(u,0);
        plotFeedforwardError(distanceIntegral*1000,...
                            (r.encoders.left-initialLocation));
        pause(0.01);
    end
    r.velocityControl(0,0);

    fprintf('Wheel movement: %dmm\n',r.encoders.left - initialLocation);
    distanceIntegral
    
    global data
    hold on;
    plot(ffPlot,1:size(data,2),data(1,:),'+r');
    plot(ffPlot,1:size(data,2),data(2,:),'+g');
    hold off;
    
end

