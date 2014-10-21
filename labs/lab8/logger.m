% rob = neato('kilo');
rob.startLaser;
input('Ready?');

while true
    log = input('Log laser data #: ');
    if log<0
        break;
    end
    filename=['Log',num2str(log)];
    ranges = rob.laser.data.ranges;
    save(filename,'ranges');
end