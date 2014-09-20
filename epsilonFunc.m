function valid = epsilonFunc(currentState, targetState, error)
% Determines if currentState is within error of targetState
    if abs(currentState.val - targetState)>error
        valid = 0;
    else
        valid = 1;
    end
end

