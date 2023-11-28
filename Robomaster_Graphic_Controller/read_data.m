clc;
clear all;

load("data.mat",mydata);

ic = data.initial_conditions;
des_val = data.desired_val;
poses = data.myposesRobomasterF;
wheels = data.mywRobomasterF;
gimball = data.mygRobomasterF;
mytime = data.mytime;
tend = data.tend;

x1d = ic.initial_conditions.x1d;
y1d = ic.initial_conditions.x1d;
th1d = ic.initial_conditions.x1d;
yaw1d = ic.initial_conditions.x1d;
pitch1d = ic.initial_conditions.x1d;


figure
hold on 
plot(mytime,tend)

figure
hold on
xRobomasterF1(:,1) = poses.myposesRobomasterF(1,1,:);
plot(mytime,xRobomasterF1)    
plot(mytime,x1d*ones(it_now,1))
title('t vs x1')

figure
hold on
yRobomasterF1(:,1) = poses.myposesRobomasterF(2,1,:);
plot(mytime,yRobomasterF1)    
plot(mytime,y1d*ones(it_now,1))
title('t vs y1')

figure
hold on
thRobomasterF1(:,1) = poses.myposesRobomasterF(3,1,:);
plot(mytime,thRobomasterF1)    
plot(mytime,th1d*ones(it_now,1))
title('t vs \theta_1')

figure
hold on
yawRobomasterF1(:,1) = poses.myposesRobomasterF(4,1,:);
plot(mytime,yawRobomasterF1)    
plot(mytime,yaw1d*ones(it_now,1))
title('t vs yaw_1')

figure
hold on
pitchRobomasterF1(:,1) = poses.myposesRobomasterF(5,1,:);
plot(mytime,pitchRobomasterF1)    
plot(mytime,pitch1d*ones(it_now,1))
title('t vs pitch_1')

figure
hold on
w1RobomasterF1(:,1) = wheels.mywRobomasterF(1,1,:);
w2RobomasterF1(:,2) = wheels.mywRobomasterF(2,1,:);
w3RobomasterF1(:,3) = wheels.mywRobomasterF(3,1,:);
w4RobomasterF1(:,4) = wheels.mywRobomasterF(4,1,:);
plot(mytime,w1RobomasterF1) 
plot(mytime,w2RobomasterF1)
plot(mytime,w3RobomasterF1)
plot(mytime,w4RobomasterF1) 
title('t vs \omega_{1,i}')

