function onNewEncoderData(src,evt,robotClass)
    robotClass.encoders = evt.data;
    
    
    
% if 0
%     obj.timeStamp = evt.data.header.stamp.secs + ...
%         (evt.data.header.stamp.nsecs/1000000000);
%     obj.leftWheelPos = obj.neatoRobot.encoders.data.left;
%     obj.rightWheelPos = obj.neatoRobot.encoders.data.right;
%     
%     if obj.prevTimeStamp ~= -1 
%         obj.dt = obj.timeStamp - obj.prevTimeStamp;
%     end
%     if obj.prevLeftWheelPos ~= -1 && obj.prevRightWheelPos ~= -1
%         obj.leftdS = obj.leftWheelPos - obj.prevLeftWheelPos;
%         obj.leftV = obj.leftdS / obj.dt;
%         
%         obj.rightdS = obj.rightWheelPos - obj.prevRightWheelPos;
%         obj.rightV = obj.rightdS / obj.dt;
%         obj.v = (obj.rightV + obj.leftV) / 2;
%         
%     end
% %     disp('LeftV')
% %     disp(obj.leftV)
% %     disp('RightV')
% %     disp(obj.rightV)
%     disp('gth');
%     disp(src);
%     obj.omega = (obj.rightV - obj.leftV) / 2;
%     
%     obj.prevTimeStamp = obj.timeStamp;
%     obj.prevLeftWheelPos = obj.leftWheelPos;
%     obj.prevRightWheelPos = obj.rightWheelPos;
% end
end