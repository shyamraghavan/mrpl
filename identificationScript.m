%% Identification script
% sends different commands to extract different values needed for the
% implementation of a feed forward controller.

%% Step response

logs = cell(7,6);

global log sec nsec logIndex
logIndex = 1;
index = 1;

for Vmax = 0.05:0.05:0.3
    r = robot('sim','Map',1);
    
    Vmax
    log  = zeros(2,1000);
    sec  = zeros(1,1000);
    nsec = zeros(1,1000);
    
    
    startRobLeft = r.rob.encoders.data.left;
    startRobRight = r.rob.encoders.data.right;
    
    start = tic;
    startRobTime = r.rob.encoders.data.header.stamp;
    logger = event.listener(r.rob.encoders,...
                    'OnMessageReceived',@responseLogger);
                
    while toc(start) < 1
        r.velocityControl(0,0);
        pause(0.1);
    end
    while toc(start) < 5
        r.velocityControl(Vmax,0);
        pause(0.1);
    end
    while toc(start) < 8
        r.velocityControl(0,0);
        pause(0.1);
    end
    disp('done.')
    r.close
    close 1
    
    
    logs{1,index}= log;
    disp('Logged left&right data');
    logs{2,index}= sec;
    disp('Logged sec data');
    logs{3,index}= nsec;
    disp('Logged nsec data');
    logs{4,index}= startRobTime.secs;
    logs{5,index}= startRobTime.nsecs;
    logs{6,index}= startRobLeft;
    logs{7,index}= startRobRight;
    
    index = index+1;
end

stepRespFig = figure(5);
hold on
box on
grid on
plot(0,0);

color = ['r' 'g' 'b' 'm' 'k' 'y'];

for index = 1:6
    startRobTimeSec = logs{4,index};
    startRobTimeNsec = logs{5,index};
    startRobLeft = logs{6,index};
    startRobRight = logs{7,index};
    time = (logs{2,index}-startRobTimeSec)+(logs{3,index}-startRobTimeNsec).*1E-9;
    cut1 = find(time>0,1,'first');
    time = time(cut1:end);
    cut2 = find(time<0,1,'first')-1;
    encoder = logs{1,index}-startRobLeft;
    if isempty(cut2)  
        encoderLeft = encoder(1,cut1:end);
        encoderRight = encoder(2,cut1:end);
    else
        time = time(1:cut2);
        encoderLeft = encoder(1,cut1:cut1+cut2-1);
        encoderRight = encoder(2,cut1:cut1+cut2-1);
    end
    hold on
    plot(time,encoderLeft,color(index));
    pause(0.5);
end

