classdef ROSCallbackData < event.EventData
    %ROSCallbackData simple callback emulates robot. 
    
    properties
        data
    end
    
    methods
        function obj = ROSCallbackData(data)
            obj.data = data;
        end
    end
    
end

