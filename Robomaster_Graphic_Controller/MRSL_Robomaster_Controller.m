%dbstop if error
clc
close all
clear all 


[config,Nmax] = init();

N.RobomasterF = 1; 

initial_conditions.RobomasterF(:,1) = [0,0,0,pi/2,0];

r = start_arena(N,initial_conditions,config); 

w.RobomasterF = zeros(4,N.RobomasterF);
g.RobomasterF = zeros(2,N.RobomasterF);


w_rad = 0.05;
L = 0.1;
W = 0.1;
M = [1, 1, (L+W);...
     1,-1,-(L+W);...
     1, 1,-(L+W);...
     1,-1, (L+W)];

%%%% CONTROLLER CONSTANTS 
k_linear = 1;
k_theta = 1;
k_gimbal = 75;


[axes, buttons, ~] = read(config.joy);


it_now = 1;
x1d = 1;
y1d = 1;
th1d = 0;
yaw1d = pi/2;
pitch1d = pi/4;

while buttons(1,2) == 0
   tstart(it_now) = tic;

   mytime(it_now) = (it_now-1)*r.Dt;
   [r,poses] = get_poses(r,N,config);

    for i = 1:N.RobomasterF

        %%%%% ROBOT CONTROL
       
        x = poses.RobomasterF(1,i);
        y = poses.RobomasterF(2,i);
        th = poses.RobomasterF(3,i);
        yaw = poses.RobomasterF(4,i);
        pitch = poses.RobomasterF(5,i);
        

        BRW = [ cos(th),sin(th),0;...
                -sin(th),cos(th),0;...
                    0    ,   0    ,1];

        w.RobomasterF(:,i) = (30/pi)*(1/w_rad)*M*BRW*[-k_linear*tanh((x-x1d));...
                                                     -k_linear*tanh((y-y1d));...
                                                     -k_theta*tanh((th-th1d))];
        
        g.RobomasterF(:,i) = [-k_gimbal*tanh((pitch-pitch1d));...
                              k_gimbal*tanh((yaw-yaw1d))];
        
        [axes, buttons, ~] = read(config.joy);
    end
    

    r = step(r,w,g,N,config); 
    mywRobomasterF(:,:,it_now) = w.RobomasterF; 
    mygRobomasterF(:,:,it_now) = g.RobomasterF;
    myposesRobomasterF(:,:,it_now) = poses.RobomasterF;

    tend(it_now) = toc(tstart(it_now));

    it_now = it_now + 1;
end  

stop(N,config)

desired_val = [x1d;y1d;th1d;yaw1d;pitch1d];

save("data.mat","initial_conditions","desired_val","myposesRobomasterF","mywRobomasterF","mygRobomasterF","mytime","tend","it_now")

it_now = it_now -1; 
for i = 1:N.RobomasterF

figure
hold on 
plot(mytime,tend)

figure
hold on
xRobomasterF1(:,1) = myposesRobomasterF(1,1,:);
plot(mytime,xRobomasterF1)    
plot(mytime,x1d*ones(it_now,1))
title('t vs x1')

figure
hold on
yRobomasterF1(:,1) = myposesRobomasterF(2,1,:);
plot(mytime,yRobomasterF1)    
plot(mytime,y1d*ones(it_now,1))
title('t vs y1')

figure
hold on
thRobomasterF1(:,1) = myposesRobomasterF(3,1,:);
plot(mytime,thRobomasterF1)    
plot(mytime,th1d*ones(it_now,1))
title('t vs \theta_1')

figure
hold on
yawRobomasterF1(:,1) = myposesRobomasterF(4,1,:);
plot(mytime,yawRobomasterF1)    
plot(mytime,yaw1d*ones(it_now,1))
title('t vs yaw_1')

figure
hold on
pitchRobomasterF1(:,1) = myposesRobomasterF(5,1,:);
plot(mytime,pitchRobomasterF1)    
plot(mytime,pitch1d*ones(it_now,1))
title('t vs pitch_1')

figure
hold on
w1RobomasterF1(:,1) = mywRobomasterF(1,1,:);
w2RobomasterF1(:,2) = mywRobomasterF(2,1,:);
w3RobomasterF1(:,3) = mywRobomasterF(3,1,:);
w4RobomasterF1(:,4) = mywRobomasterF(4,1,:);
plot(mytime,w1RobomasterF1) 
plot(mytime,w2RobomasterF1)
plot(mytime,w3RobomasterF1)
plot(mytime,w4RobomasterF1) 
title('t vs \omega_{1,i}')

end

