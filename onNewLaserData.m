%% OnNewLaserData
function onNewLaserData(~, laser)

laserPlotPol  = get(findobj('Tag','laserFigPol'),'Children');
laserPlotCart = get(findobj('Tag','laserFigCart'),'Children');

cleanedLaserRanges = rangeDataFilter(laser.data.ranges);

%laserPlotPol(2) is the data line object, while laserPlotpol(1) is the
%sim robot image.
set(laserPlotPol(2),...
 	'XData',cleanedLaserRanges.*cos((90:449).*(pi/180)),...
	'YData',cleanedLaserRanges.*sin((90:449).*(pi/180)));

set(laserPlotCart,...
	'YData',laser.data.ranges);

end