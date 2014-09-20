function pError = pErrorFunc(kp, pCurrentState, pTargetState)
%proportional error function
pError = (pTargetState - pCurrentState.val)*kp;
end