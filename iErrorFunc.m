function iError = iErrorFunc(ki, iCurrentState, iTargetState, integralError)
%integral error function
iError = ((iTargetState - iCurrentState.val) + integralError)*ki;
end

