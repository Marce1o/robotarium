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
state_publisher = rospublisher("/exec_state","std_msgs/String","DataFormat","struct");
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
n_robots = length(robot_names);


dx = zeros(n_robots);
dy = zeros(n_robots);
dx(1) = 0;
dy(1) = 1;


dtheta = zeros(n_robots);
dtheta(1) = 0;
dtheta(2) = 0;

w_rad = 0.05;
L = 0.1;
W = 0.1;
M = [1, 1, (L+W);...
     1,-1,-(L+W);...
     1, 1,-(L+W);...
     1,-1, (L+W)];

%%%% GIMBAL VARIABLES
xpd = zeros(n_robots);
xyd = zeros(n_robots);

%%%% CONTROLLER CONSTANTS 
max_linear = 0.5;
max_theta = 2.4906586;
max_gimbal_speed = 90;


for i = 1:n_robots
    sub_topic = "/vicon/"+robot_names(i)+"/"+robot_names(i);
    subscriber = rossubscriber(sub_topic,"geometry_msgs/TransformStamped");
    robot_topics(robot_names(i)) = subscriber;   
    pause(1)

    sub_topic = "/vicon/gimbal_"+robot_names(i)+"/gimbal_"+robot_names(i);
    subscriber = rossubscriber(sub_topic,"geometry_msgs/TransformStamped");
    gimbal_topics(robot_names(i)) = subscriber;
    pause(1)


    wheel_publishers(robot_names(i)) = rospublisher(string(robot_names(i))+'/wheels',"geometry_msgs/Quaternion");
    pause(0.5)

    gimbal_publishers(robot_names(i)) = rospublisher(string(robot_names(i))+'/gimbal_speed',"geometry_msgs/Point");
    pause(0.5)
end

[axes, buttons, ~] = read(joy);


k = 1;

while buttons(1,2) == 0

    time(k) = (k-1)*dt;

    robot_speeds = '';


    for i = 1:n_robots

        %%%%% ROBOT CONTROL
        pose_data = receive(robot_topics(robot_names(i)),10);
        gimbal_data = receive(gimbal_topics(robot_names(i)),10);


        robot_pose = getPose(pose_data);
        [gimbal_orientation,pose_flag] = getOrientation(gimbal_data);
        
        xpos(k,i) = robot_pose(1);
        ypos(k,i) = robot_pose(2);
        thetapos(k,i) = robot_pose(4);
        
        ux = speed_controller(robot_pose(1),dx(i),max_linear);
        uy = speed_controller(robot_pose(2),dy(i),max_linear);
        uw = speed_controller(robot_pose(4),dtheta(i),max_theta);

        %speed_vec = [-uy;-ux;uw];

        speed_vec = [ux;uy;uw];
        
        wheelSpeed = matrix*speed_vec;

        wheelSpeed = wheelSpeed*(30/pi);

        robot_temp = transpose(wheelSpeed);

        angle_offset = gimbal_orientation(1) -  rad2deg(robot_pose(4));
        [axes, buttons, ~] = read(joy);
        uy = speed_controller_gimbal(xyd(i),angle_offset,max_gimbal_speed);
        up = speed_controller_gimbal(xpd(i),gimbal_orientation(2),max_gimbal_speed);

        yawpos(k,i) = gimbal_orientation(1);
        pitchpos(k,i) = gimbal_orientation(2);
        
        gimbal_temp = [up,-uy];  
        send_twist(wheel_publishers(robot_names(i)),robot_temp)
        send_point(gimbal_publishers(robot_names(i)),gimbal_temp)

    end
    
    pause(dt)
    k = k + 1;


end  
send_twist(wheel_publishers(robot_names(i)),[0,0,0,0])
send_point(gimbal_publishers(robot_names(i)),[0,0])
send_ros(state_publisher,'shutdown')
rosshutdown

figure
plot(time,xpos)
title('x-position')
legend(robot_names)

figure
plot(time,ypos)
title('y-position')
legend(robot_names)

figure
plot(time,thetapos)
title('theta-position')
legend(robot_names)

figure
plot(time,yawpos)
title('yaw-position')
legend(robot_names)

figure
plot(time,pitchpos)
title('pitch-position')
legend(robot_names)

figure
hold on
fill([6.5,-6.5,-6.5,-3.14,6.5],[2.5,2.5,0,-2.5,-2.5],[200/255, 200/255, 200/255])
axis equal
plot(xpos,ypos)
title('x-y pos')
legend(robot_names)


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

function [orientation,flag] = getOrientation(data)

        angle_x = data.Transform.Rotation.X;
        angle_y = data.Transform.Rotation.Y;
        angle_z = data.Transform.Rotation.Z;
        angle_w = data.Transform.Rotation.W;

        [yaw,pitch,roll] = quat2angle([angle_x angle_y angle_z angle_w]);
        yaw_d = rad2deg(roll); %horizontal

        pitch_d = rad2deg(pitch); %vertical
        orientation = [yaw_d,pitch_d];

        if yaw_d < 0 
            flag = 'positive';
        
        elseif yaw_d >= 0
            flag = 'negative';
        
        end
        

end 

function u = speed_controller(c,d,k1)
    e = (c - d);
    %disp(-200*tanh(e))
    u = -k1*tanh(2*e);

end
function u = speed_controller_gimbal(c,d,k1)
    e = (c - d)/(k1-20);
    %disp(-200*tanh(e))
    u = k1*tanh(e);

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