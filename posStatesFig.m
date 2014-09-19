%% Possible Path Envelop visualizer
% for a given v, the envelope for the possible future states.
v = 0.15;
t = 0:0.01:2.5;
wMax = 0.3/0.2350; 
wMin = -wMax;

deltaS = v.*t;

index = 0;
x = zeros(100,251);
y = zeros(100,251);

for w = wMin:0.0255:wMax; 
index = index+1;
deltaTh = w.*t;
r = deltaS./deltaTh;
x(index,:) = r.*(1-cos(deltaTh));
y(index,:) = r.*sin(deltaTh);
end

cc = jet(100);
hold on
axis equal
for k = 1:100
    plot(x(k,:),y(k,:),'-','color',cc(k,:))
end


