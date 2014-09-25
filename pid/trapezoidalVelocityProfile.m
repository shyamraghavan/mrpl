function uref = trapezoidalVelocityProfile( t , amax, vmax, dist)
%uref Return the velocity command of a trapezoidal profile.
% Returns the velocity command of a trapezoidal profile of maximum
% acceleration amax and maximum velocity vmax whose ramps are of 
% duration tf. Sgn is the sign of the desired velocities. 
% Returns 0 if t is negative.
% global debug
debug = false;
% time to ramp (accellerate)
tRamp = vmax / amax;

if dist > 0
    sgn = true;
else
    sgn = false;
end

% The distance travelled while ramping velocity, done at beginning and end.
sRamp = tRamp*vmax/2;
% Check my math here. I am getting the distance travelled at max velocity.
sf = abs(dist)-2*sRamp;
tf = sf/vmax + 2*tRamp;

if debug
    fprintf('Ramp Time: \t%d seconds\n',tRamp);
    fprintf('Ramp Dist: \t%d mm \n',sRamp);
    fprintf('Max Velocity Dist: %d mm \n',sf);
    fprintf('Total Time: \t%d seconds\n',tf);
end
% The algorithm described at the beginning of 3.3 Warm up Exercise 3.
if t <= tRamp
    if sgn
        uref = amax * t;
    else
        uref = -amax * t;
    end
elseif 0 <= (tf - t) && (tf - t) < tRamp
    if sgn
        uref = amax * (tf - t);
    else
        uref = -amax * (tf - t);
    end
elseif tRamp < t && t <= (tf-tRamp)
    if sgn
        uref = vmax;
    else
        uref = -vmax;
    end
elseif tf <= t
    uref = 0;
else
    uref = 0;
end

if debug
    fprintf('uref: \t%d m/s\n',uref);
end
end