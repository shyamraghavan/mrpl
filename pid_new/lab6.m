%% Lab 6

rob = robot('sim');
controller = pidController(rob);
tablesBuilt = true;
if ~tablesBuilt
    cubicSpiral.makeLookupTable(50);
end
