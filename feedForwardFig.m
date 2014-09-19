t = 0:0.01:5;
v = zeros(1,501);
x = zeros(1,501);

for index=1:501
    v(index)=trapezoidalVelocityProfile(t(index),0.75,0.25,1,1);
    if index < 501
        x(index+1)= x(index)+v(index)*0.01;
    end
end

hold on
plot(t,v);
plot(t,x);
