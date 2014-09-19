function responseLogger( ~, encoders)
%UNTITLED6 Summary of this function goes here
%   Detailed explanation goes here

global log sec nsec logIndex

log(1,logIndex)= encoders.data.left;
log(2,logIndex)= encoders.data.right;
sec(1,logIndex)= encoders.data.header.stamp.secs;
nsec(1,logIndex)= encoders.data.header.stamp.nsecs;

logIndex = logIndex+1;
end

