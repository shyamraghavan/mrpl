%TODO: measure the offset and make sure i got the goto fucntion correct

offset = 1;
h = plot(0,0);
while true
    ranges = rob.laser.data.ranges;
    rangeImg = rangeImage(ranges,1,true);
    rangeImg.plotXvsY(h);
    
    
    errMax = 1000;
    numMax = 1;
    for i = 1:img.numPix
        [err, num, th] = findLineCandidate(img,i,0.125);

        if num > numMax || (num == numMax && err<errMax)
            lines(n,:) = [i,num,th,err];
            n=n+1;
            errMax = err;
            numMax = num;
            subplot(1,2,2)
            plot([img.xArray(mod((lines(end,1)-1)-lines(end,2),img.numPix)+1),
            img.xArray(mod((lines(end,1)-1)+lines(end,2),img.numPix)+1)],...
            [img.yArray(mod((lines(end,1)-1)-lines(end,2),img.numPix)+1),
            img.yArray(mod((lines(end,1)-1)+lines(end,2),img.numPix)+1)],'ko-');
            xlim([-2.0,2.0])
            ylim([-2.0,2.0])
            grid on
            hold on
        end
    end
    plot(img.xArray(mod((lines(end,1)-1)-lines(end,2),img.numPix)+1:mod((lines(end,1)-1)+lines(end,2),img.numPix)+1),...
    img.yArray(mod((lines(end,1)-1)-lines(end,2),img.numPix)+1:mod((lines(end,1)-1)+lines(end,2),img.numPix)+1),'r-','Linewidth',3);

    [targetX,targetY]=rangeimg.getTarget(lines(end,1),lines(end,3),offset);
    goTo(rob,targetX,targetY,bestLine(3),1);
    input('Go to next target? ');
end