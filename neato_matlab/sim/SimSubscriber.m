classdef SimSubscriber < handle
	events
        OnMessageReceived % Nofified when message is received by this subscriber
    end
    
    properties
        data
	end
    
    methods
		function obj = SimSubscriber(data)
			obj.data = data;
		end
        function publish(obj)
			notify(obj, 'OnMessageReceived',ROSCallbackData(obj.data));
        end
    end 
end 

