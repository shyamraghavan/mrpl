function logLaserData( ~,laser )
%writes the data from the laser to a csv file with time stamp info.
    global logIndex laserLoggingListener;
    maxLog = 100;

    ranges = laser.data.ranges;
    timeS = laser.data.header.stamp.secs;
    timeNsecs = laser.data.header.stamp.nsecs;
    
    data = [timeS,timeNsecs,0,1,ranges];
    dlmwrite('laserData.csv',data,'-append');
    
    if logIndex < maxLog
        logIndex = logIndex+1;
    else
        delete(laserLoggingListener);
    end
end