function lab4(ff)
    if ff
        clear all;
        close all;
        clc; 
        pidSetup;
        PID.runFF(1000,2);
    else
        clear all;
        close all;
        clc; 
        testFeedForward;
    end
end