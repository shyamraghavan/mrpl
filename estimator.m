function estimator(~, data)
    global robot
    obj = robot;

    leftPast = obj.encoders.leftPast;
    rightPast=obj.encoders.rightPast;
    timePastSec = obj.encoders.timePastSec;
    timePastNSec = obj.encoders.timePastNSec;
    
    leftCurrent = data.left;
    rightCurrent = data.right;
    timeCurrentSec = data.header.stamp.secs;
    timeCurrentNSec = data.header.stamp.nsecs;
    
    deltaTime = (timeCurrentSec+timeCurrentNSec*1E-9)-(timePastSec+timePastNSec*1E-9);
    deltaLeft = leftCurrent-leftPast;
    deltaRight = rightCurrent-rightPast;
    
    velocity = (deltaLeft+deltaRight)/(2*deltaTime);
    omega = (deltaLeft-deltaRight)/(obj.wheelbase*deltaTime);
    
    obj.deadReckoning.thPos = obj.deadReckoning.thPos...
                              + (omega*deltaTime)/2;
    obj.deadReckoning.xPos = obj.deadReckoning.xPos...
                              + velocity*deltaTime*cos(obj.deadReckoning.thPos);
    obj.deadReckoning.yPos = obj.deadReckoning.yPos ...
                              + velocity*deltaTime*sin(obj.deadReckoning.thPos);
    obj.deadReckoning.thPos = obj.deadReckoning.thPos...
                              + (omega*deltaTime)/2;
    
    obj.encoders.leftPast = leftCurrent;
    obj.encoders.rightPast = rightCurrent ;
    obj.encoders.timePastSec = timeCurrentSec;
    obj.encoders.timePastNSec =  timeCurrentNSec;
    

end