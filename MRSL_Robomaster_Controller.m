%dbstop if error
clc
close all
clear all 


[config,Nmax] = init();

N.RobomasterF = 2; 

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

        yaw = poses.GimbalF(1,i);
        pitch = poses.GimbalF(2,i);
        

        BRW = [ cos(th),sin(th),0;...
                -sin(th),cos(th),0;...
                    0    ,   0    ,1];

        w.RobomasterF(:,i) = (1/wrad)*M*BRW*[-k_linear*tanh(2*(x-xd));...
                                             -k_linear*tanh(2*(y-yd));...
                                             -k_theta*tanh(2*(th-thd))];
        
        w.RobomasterF(:,i) =  w.RobomasterF(:,i)*(30/pi);


        g.RobomasterF(:,i) = [k_gimbal*tanh((yaw-yawd)/k_gimbal);...
                              k_gimbal*tanh((pitch-pitchd)/k_gimbal)];
      

        [axes, buttons, ~] = read(config.joy);
        send_twist(r.r_pub(r.r_names(i)),robot_temp)
        send_point(r.g_pub(r.r_names(i)),gimbal_temp)

    end

    mywRobomasterF(:,:,it_now) = w.RobomasterF; 
    myposesRobomasterF(:,:,it_now) = poses.RobomasterF;
    it_now(k) = (k-1)*r.Dt;
    k = k + 1;
    
end  

stop(N,config)


figure
plot(it_now,xpos)
title('x-position')
legend(robot_names)

figure
plot(it_now,ypos)
title('y-position')
legend(robot_names)

figure
plot(it_now,thetapos)
title('theta-position')
legend(robot_names)

figure
plot(it_now,yawpos)
title('yaw-position')
legend(robot_names)

figure
plot(it_now,pitchpos)
title('pitch-position')
legend(robot_names)

figure
hold on
fill([6.5,-6.5,-6.5,-3.14,6.5],[2.5,2.5,0,-2.5,-2.5],[200/255, 200/255, 200/255])
axis equal
plot(xpos,ypos)
title('x-y pos')
legend(robot_names)
