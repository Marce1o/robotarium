%dbstop if error
clc
close all
clear all 


[config,Nmax] = init();

N.RobomasterF = 1; 

initial_conditions.RobomasterF(:,1) = [0,0,0,0,0];

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

while buttons(1,2) == 0
    
   [r,poses] = get_poses(r);

    for i = 1:n_robots

        %%%%% ROBOT CONTROL
       
        x = poses.RobomasterF(1,i);
        y = poses.RobomasterF(2,i);
        th = poses.RobomasterF(3,i);
        yaw = poses.RobomasterF(4,i);
        pitch = poses.RobomasterF(5,i);
        

        BRW = [ cos(th),sin(th),0;...
                -sin(th),cos(th),0;...
                    0    ,   0    ,1];

        w.RobomasterF(:,i) = (1/wrad)*M*BRW*[-k_linear*tanh(2*(x-xd));...
                                             -k_linear*tanh(2*(y-yd));...
                                             -k_theta*tanh(2*(th-thd))];
        
        w.RobomasterF(:,i) =  w.RobomasterF(:,i)*(30/pi);


        g.RobomasterF(:,i) = [k_gimbal*tanh((yaw-yawd));...
                              k_gimbal*tanh((pitch-pitchd))];
        

        [axes, buttons, ~] = read(config.joy);
        send_twist(r.r_pub(r.r_names(i)),robot_temp)
        send_point(r.g_pub(r.r_names(i)),gimbal_temp)

    end
    

    r = step(r,w,N,config); 
    mywRobomasterF(:,:,it_now) = w.RobomasterF; 
    myposesRobomasterF(:,:,it_now) = poses.RobomasterF;
    it_now(k) = (k-1)*r.Dt;
    k = k + 1;
    
end  

stop(N,config)


for i 1:N.RobomasterF

figure
hold on
xRobomasterF1(:,1) = myposesRobomasterF(1,1,:);
plot(mytime,xRobomasterF1)    
plot(mytime,x1d*ones(it_max,1))
title('t vs x1')

figure
hold on
yRobomasterF1(:,1) = myposesRobomasterF(2,1,:);
plot(mytime,yRobomasterF1)    
plot(mytime,y1d*ones(it_max,1))
title('t vs y1')

figure
hold on
thRobomasterF1(:,1) = myposesRobomasterF(3,1,:);
plot(mytime,thRobomasterF1)    
plot(mytime,th1d*ones(it_max,1))
title('t vs \theta_1')

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

figure
hold on
xRobomasterF2(:,1) = myposesRobomasterF(1,2,:);
plot(mytime,xRobomasterF2)    
plot(mytime,x2d*ones(it_max,1))
title('t vs x2')

figure
hold on
yRobomasterF2(:,1) = myposesRobomasterF(2,2,:);
plot(mytime,yRobomasterF2)    
plot(mytime,y2d*ones(it_max,1))
title('t vs y2')

figure
hold on
thRobomasterF2(:,1) = myposesRobomasterF(3,2,:);
plot(mytime,thRobomasterF2)    
plot(mytime,th2d*ones(it_max,1))
title('t vs \theta_2')

figure
hold on
w1RobomasterF2(:,1) = mywRobomasterF(1,2,:);
w2RobomasterF2(:,2) = mywRobomasterF(2,2,:);
w3RobomasterF2(:,3) = mywRobomasterF(3,2,:);
w4RobomasterF2(:,4) = mywRobomasterF(4,2,:);
plot(mytime,w1RobomasterF2) 
plot(mytime,w2RobomasterF2)
plot(mytime,w3RobomasterF2)
plot(mytime,w4RobomasterF2) 
title('t vs \omega_{2,i}')


end

