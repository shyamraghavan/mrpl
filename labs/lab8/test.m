walls = lineObject();

              % x  y
walls.lines = [-2.5 -2.5;
                2.5 -2.5;
                2.5  2.5; 
               -2.5  2.5; 
               -2.5 -2.5];
rob=neato('sim');
rob.startLaser;
% img;
vl = [0.20, 0.15, 0.20, 0.15];
vr = [0.20, 0.30, 0.20, 0.30];
pose = [1, 1.5,   0.75, -0.4 ;...%x
        1, -0.5, -0.5,  0.3 ;...%y
	    0, pi/6,   pi/2, 4*pi/5 ];
pose = pose';


r = sqrt(0.038^2+0.0625^2);
phi = atan(0.125/0.038);
ob1 = lineObject();
ob1.lines = [r*cos(phi)   , r*sin(phi);...
             r*cos(pi-phi), r*sin(pi-phi);...
             r*cos(pi+phi), r*sin(pi+phi);...
             r*cos(-phi)  , r*sin(-phi);...
             r*cos(phi)   , r*sin(phi)];
for step = 1:4
    

    if step ==1
        map = lineMap([walls ob1]);
        rob.genMap(map.objects);
    end
    rob.map.objects(2).pose=pose(step,:);
    pause(1)
    ranges = rob.laser.data.ranges;
    rangeImg = rangeImage(ranges,1,true);
    img(step)=rangeImg;
    step
end
rob.shutdown;
figure
hold on
str = '+r+g+m+b';
col = jet(40);
for step = 1:4
   rangeImg = img(step);
   errMax = 1000;
   bestLine = [0,0,0];
   n = 1;
   for i = 1:rangeImg.numPix
      [err, num, th] = findLineCandidate(rangeImg,i,0.125)
      num
      if num > 0
          bestLine(n,:) = [i,num,th*180/pi];
          n=n+1;
          errMax = err;
          plot([rangeImg.xArray(mod((bestLine(end,1)-1)-bestLine(end,2),rangeImg.numPix)+1),
                rangeImg.xArray(mod((bestLine(end,1)-1)+bestLine(end,2),rangeImg.numPix)+1)],...
               [rangeImg.yArray(mod((bestLine(end,1)-1)-bestLine(end,2),rangeImg.numPix)+1),
                rangeImg.yArray(mod((bestLine(end,1)-1)+bestLine(end,2),rangeImg.numPix)+1)],'ko-','Color',col(n,:));
      end
   end
   bestLine
   plot(rangeImg.xArray,rangeImg.yArray,str(1+2*(step-1):2*step));
   plot(rangeImg.xArray(mod((bestLine(end,1)-1)-bestLine(end,2),rangeImg.numPix)+1:mod((bestLine(end,1)-1)+bestLine(end,2),rangeImg.numPix)+1),...
        rangeImg.yArray(mod((bestLine(end,1)-1)-bestLine(end,2),rangeImg.numPix)+1:mod((bestLine(end,1)-1)+bestLine(end,2),rangeImg.numPix)+1),'k-','Linewidth',3);
end