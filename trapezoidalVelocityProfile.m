function [uref] = trapezoidalVelocityProfile( t, aMax, vMax, dist, sgn)
% uref Return the velocity command of a trapezoidal profile.
% Returns the velocity command of a trapezoidal profile of maximum
% acceleration amax and maximum velocity vmax whose ramps are of 
% duration tRamp. Sgn is the sign of the desired velocities. 
% Returns 0 if t is negative. 

    tRamp = vMax/aMax;
    tf = (dist-aMax*tRamp^2)/vMax+2*tRamp;

    if (t < 0)
        uref = 0;
        return;
    elseif t < tRamp
        uref = aMax*sgn.*t;
        return;
    elseif t < tf - tRamp
        uref = vMax;
        return;
    elseif t < tf
        uref = -aMax*sgn.*(t-tf);
        return;
    else
        uref = 0;
        return;
    end
end

