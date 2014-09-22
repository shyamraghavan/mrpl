classdef laserPolarPlot < handle
    properties
        cleanedLaserRanges;
    end
    methods
    
        function obj = laserPolarPlot()
            laserPlotPol  = get(findobj('Tag','laserFigPol'),'Children');
            laserPlotCart = get(findobj('Tag','laserFigCart'),'Children');

            obj.cleanedLaserRanges = rangeDataFilter(evt.data.ranges);
            [i,r] = nearestObject(inSector(obj.cleanedLaserRanges, -45, 45));


            if i ~= 1
                [x,y,~] = irToXy(i + 90,r);
                set(laserPlotPol(1),...
                'XData',x,...
                'YData',y);
            end

            %laserPlotPol(2) is the data line object, while laserPlotpol(1) is the
            %sim robot image.
            set(laserPlotPol(3),...
                'XData',obj.cleanedLaserRanges.*cos((90:449).*(pi/180)),...
                'YData',obj.cleanedLaserRanges.*sin((90:449).*(pi/180)));

            set(laserPlotCart,...
                'XData',1:360,...
                'YData',[fliplr(evt.data.ranges(1:180)),fliplr(evt.data.ranges(181:360))]);
        end
    end
end