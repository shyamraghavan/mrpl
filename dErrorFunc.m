function dError = dErrorFunc(kd, dCurrentState, dTargetState, dPreviousError)
%proportional error function
dError = struct('val',-1,...
                'time',-1);
dError.val = (((dTargetState - dCurrentState.val) - dPreviousError.val)/...
              (dCurrentState.time -dPreviousError.time))*kd;
dError.time = dCurrentState.tim;
end

