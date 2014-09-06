function milliSeconds = milli(timeStamp)
% conversts the header time stamp into a simple milli second value
secs = timeStamp.secs;
nsecs = timeStamp.nsecs;

milliSeconds = secs*1E3 + nsecs*1E-6;
end

