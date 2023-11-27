%dbstop if error
clc
close all
clear all 


[config,Nmax] = init();

N.RobomasterF = 1; 

initial_conditions.RobomasterF(:,1) = [0,0,0,1.57,0];

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
k_linear = 0.5;
k_theta = 2.4906586;
k_gimbal = 90;


[axes, buttons, ~] = read(config.joy);


k = 1;
x1d = 1;
y1d = 1;
th1d = 0;
yaw1d = 1.57;
pitch1d = pi/4;

while buttons(1,2) == 0
   it_now(k) = (k)*r.Dt;

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
    mywRobomasterF(:,:,k) = w.RobomasterF; 
    mygRobomasterF(:,:,k) = g.RobomasterF;
    myposesRobomasterF(:,:,k) = poses.RobomasterF;

    k = k + 1;
    
end  

stop(N,config)


for i = 1:N.RobomasterF

figure
hold on
xRobomasterF1(:,1) = myposesRobomasterF(1,1,:);
plot(it_now,xRobomasterF1)    
plot(it_now,x1d*ones(it_now,1))
title('t vs x1')

figure
hold on
yRobomasterF1(:,1) = myposesRobomasterF(2,1,:);
plot(it_now,yRobomasterF1)    
plot(it_now,y1d*ones(it_now,1))
title('t vs y1')

figure
hold on
thRobomasterF1(:,1) = myposesRobomasterF(3,1,:);
plot(it_now,thRobomasterF1)    
plot(it_now,th1d*ones(it_now,1))
title('t vs \theta_1')

figure
hold on
yawRobomasterF1(:,1) = mygRobomasterF(1,1,:);
plot(it_now,yawRobomasterF1)    
plot(it_now,yaw1d*ones(it_now,1))
title('t vs \yaw_1')

figure
hold on
pitchRobomasterF1(:,1) = mygRobomasterF(2,1,:);
plot(it_now,pitchRobomasterF1)    
plot(it_now,th1d*ones(it_now,1))
title('t vs \pitch_1')

figure
hold on
w1RobomasterF1(:,1) = mywRobomasterF(1,1,:);
w2RobomasterF1(:,2) = mywRobomasterF(2,1,:);
w3RobomasterF1(:,3) = mywRobomasterF(3,1,:);
w4RobomasterF1(:,4) = mywRobomasterF(4,1,:);
plot(it_now,w1RobomasterF1) 
plot(it_now,w2RobomasterF1)
plot(it_now,w3RobomasterF1)
plot(it_now,w4RobomasterF1) 
title('t vs \omega_{1,i}')

end

