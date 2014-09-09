function PlotFunction
    set(get(gca,'Children'),...
	'XData',e.data.ranges.*cos((0:359).*(pi/180)),...
	'YData',e.data.ranges.*sin((0:359).*(pi/180)))
end

%% Test Function

function x = test(n)
fprintf('Hello World!\n')
x = true;
end

%% Call Test