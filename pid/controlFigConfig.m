%% Control Fig Config
function controlFigConfig()
    figure();
    errorPlot = subplot(2,1,1);
    plot(errorPlot,1:10,1:10,'+m');
    title(errorPlot,'Distance Error (m)');
    set(errorPlot,'Tag','feedForwardErrorPlot',...
                  'YMinorGrid', 'on');
    ffPlot = subplot(2,1,2);
    hold on;
    ffPlot1 = plot(ffPlot,1:10,1:10,'+r');
    ffPlot2 = plot(ffPlot,1:10,10:-1:1,'+g');
    hold off;
    title(ffPlot,'Distance Error (m)');
    set(ffPlot1,'Tag','feedForwardPlot1');
    set(ffPlot2,'Tag','feedForwardPlot2');
end
