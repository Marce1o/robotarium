%dbstop if error
clc
close all
clear all 

rosshutdown 

rosinit 

ID = 1;
joy=vrjoystick(ID);


updatefreq = 30;
dt = 1/updatefreq;

%%%% ROS PUBLISHERS 
state_publisher = rospublisher("/state","std_msgs/String","DataFormat","struct");
wheel_publishers = dictionary; 
gimbal_publishers = dictionary;

%%%% ROS SUBSCRIBERS 
active_robots = rossubscriber("/robot_names","std_msgs/String","DataFormat","struct");
pause(1)
robot_topics = dictionary; 
gimbal_topics = dictionary;

%%%% ROBOT VARIABLES 
%robot_names = receive(active_robots,100);
%robot_names = robot_names.Data

robot_names = "rm_3";
robot_names = split(robot_names);

robot_number = length(robot_names);


dx = zeros(robot_number);
dy = zeros(robot_number);
dx(1) = 0;
dy(1) = 0;
dtheta = zeros(robot_number);
%dtheta(1) = 1.57;

wheel_radius = 0.05;
robot_l = 0.10;
robot_w = 0.10;

%matrix = 1/wheel_radius*[1 -1 -(robot_l+robot_w); 1 1 (robot_l+robot_w); 1 -1 (robot_l+robot_w); 1 1 -(robot_l+robot_w)]
matrix = 1/wheel_radius*[1 1 (robot_l+robot_w); 1 -1 -(robot_l+robot_w); 1 1 -(robot_l+robot_w); 1 -1 (robot_l+robot_w)];

%%%% GIMBAL VARIABLES4906586
xpd = zeros(robot_number);
xyd = zeros(robot_number);

max_gimbal_speed = 180;


%%%% SPEED CONSTANTS 
max_linear = 0.5;
max_theta = 2.4906586;

for i = 1:robot_number
    sub_topic = "/vicon/"+robot_names(i)+"/"+robot_names(i);
    subscriber = rossubscriber(sub_topic,"geometry_msgs/TransformStamped");
    robot_topics(robot_names(i)) = subscriber;   
    pause(1)

    sub_topic = "/"+robot_number(i)+"/gimbal";
    subscriber = rossubscriber(sub_topic,"geometry_msgs/Point");
    gimbal_topics(robot_names(i)) = subscriber;
    pause(1)


    wheel_publishers(robot_names(i)) = rospublisher(string(robot_names(i))+'/wheels',"geometry_msgs/Quaternion");
    pause(0.5)

    gimbal_publishers(robot_number(i)) = rospublisher(string(robot_names(i))+'/gimbal_speed',"geometry_msgs/Point");
    pause(0.5)
end

[axes, buttons, ~] = read(joy);


k = 1;

while buttons(1,2) == 0

    time(k) = (k-1)*dt;

    robot_speeds = '';


    for i = 1:robot_number

        %%%%% ROBOT CONTROL
        pose_data = receive(robot_topics(robot_names(i)),10);
        gimbal_data = receive(gimbal_topics(robot_names(i)),10);


        robot_pose = getPose(pose_data);
        
        xpos(k) = robot_pose(1);
        ypos(k) = robot_pose(2);
        thetapos(k) = robot_pose(4);
        
        ux = speed_controller(robot_pose(1),dx(robot_number),max_linear);
        uy = speed_controller(robot_pose(2),dy(robot_number),max_linear);
        uw = speed_controller(robot_pose(4),dtheta(robot_number),max_theta);

        %speed_vec = [-uy;-ux;uw];

        speed_vec = [ux;uy;uw];

        wheelSpeed = matrix*speed_vec;

        wheelSpeed = wheelSpeed*(30/pi);

        robot_temp = transpose(wheelSpeed);

      
        [axes, buttons, ~] = read(joy);
        xp = gimbal_data.x;
        xy = gimbal_data.y;
        up = control_calc(xpd(i),xp,max_gimbal_speed);
        uy = control_calc(xyd(i),xy,max_gimbal_speed);
        gimbal_temp = [up,uy];
         
        send_twist(wheel_publishers(robot_names(i)),robot_temp)
        send_point(gimbal_publishers(robot_names(i)),gimbal_temp)

    end
    
    %%%%%%%%%%%%%%%%%%%%%%% ENVIA INFORMACION 
    %%%% ENVIA ROBOTS
    %send_ros(pub_wheel_speed,robot_speeds)

    % %%%% ENVIA GIMBAL
    % send_ros(pub_gimbal_speed,gimbal_speeds)
    pause(dt)
    k = k + 1;


end  

send_ros(state_publisher,'shutdown')
rosshutdown

figure
plot(time,xpos)
title('x-position')

figure
plot(time,ypos)
title('y-position')

figure
plot(time,thetapos)
title('theta-position')


function pose = getPose(data)

        pose_x = data.Transform.Translation.X;
        pose_y = data.Transform.Translation.Y;
        pose_z = data.Transform.Translation.Z;

        angle_x = data.Transform.Rotation.X;
        angle_y = data.Transform.Rotation.Y;
        angle_z = data.Transform.Rotation.Z;
        angle_w = data.Transform.Rotation.W;

        [yaw,pitch,roll] = quat2angle([angle_x angle_y angle_z angle_w]);
        roll_d = rad2deg(roll);
        %disp(roll_d)
        pose = [pose_x pose_y pose_z roll];

end 

function u = speed_controller(c,d,k1)
    e = (c - d);
    %disp(-200*tanh(e))
    u = -k1*tanh(2*e);

end

function send_ros(topic,data)
    pub_msg = rosmessage(topic);
    pub_msg.Data = data;
    send(topic,pub_msg)
    
end

function send_twist(topic,data)
    msg = rosmessage(topic);
    msg.X = data(1);
    msg.Y = data(2);
    msg.Z = data(3);
    msg.W = data(4);

    send(topic,msg)
end


function send_point(topic,data)
    msg = rosmessage(topic);
    msg.X = data(1);
    msg.Y = data(2);

    send(topic,msg)
end