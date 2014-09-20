%% PID Controller Setup for distance

rob = robot('sim','Map',1,'Laser');

PID = pidController(rob);

setPGain(PID,1);
setIGain(PID,1);
setDGain(PID,1);

setPErrorFunc(PID,@pErrorFunc); 
setIErrorFunc(PID,@iErrorFunc);
setDErrorFunc(PID,@dErrorFunc);

setEpsilonFunc(PID,@epsilonFunc);
setCurrentStateFunc(PID,@currentStateFunc);
setPidOutputFunc(PID,@pidOutFunc);