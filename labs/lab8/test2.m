
close all

color = jet(10);
subplot(1,2,1)
plot(0,0)
hold on
xlim([-2.0,2.0])
ylim([-2.0,2.0])
grid on

for x = 3:9
    subplot(1,2,1)
    filename = ['Log',num2str(x+1),'.mat'];
    load(filename);
    img = rangeImage(ranges,1,true);
    
    plot(img.xArray,img.yArray,'Color',color(x+1,:),'Marker','+','LineStyle','none');
    subplot(1,2,2)
    grid on
    
    n= 1;
    lines = [0,0,0,0];
    errMax = 1000;
    numMax = 1;
    col = copper(40);
    for i = 1:img.numPix
        [err, num, th] = findLineCandidate(img,i,0.125);

        if num > numMax || (num == numMax && err<errMax)
            lines(n,:) = [i,num,th*180/pi,err];
            n=n+1;
            errMax = err;
            numMax = num;
            plot([img.xArray(mod((lines(end,1)-1)-lines(end,2),img.numPix)+1),
            img.xArray(mod((lines(end,1)-1)+lines(end,2),img.numPix)+1)],...
            [img.yArray(mod((lines(end,1)-1)-lines(end,2),img.numPix)+1),
            img.yArray(mod((lines(end,1)-1)+lines(end,2),img.numPix)+1)],'o-','Color',col(n,:));
            xlim([-2.0,2.0])
            ylim([-2.0,2.0])
            grid on
            hold on
        end
    end
    plot(img.xArray(mod((lines(end,1)-1)-lines(end,2),img.numPix)+1:mod((lines(end,1)-1)+lines(end,2),img.numPix)+1),...
    img.yArray(mod((lines(end,1)-1)-lines(end,2),img.numPix)+1:mod((lines(end,1)-1)+lines(end,2),img.numPix)+1),'r-','Linewidth',3);
input('next');
end