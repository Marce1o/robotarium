%dbstop if error
clc
close all
clear all 

rosshutdown 

rosinit 

ID = 1;
joy=vrjoystick(ID);


%%%% ROS PUBLISHERS 
state_publisher = rospublisher("/state","std_msgs/String","DataFormat","struct");


%%%% ROS SUBSCRIBERS 
active_robots = rossubscriber("/robot_names","std_msgs/String","DataFormat","struct");
pause(1)
% gimbal_state = rossubscriber("/gimbal_state","std_msgs/String","DataFormat","struct");
% pause(1)


%%%% ROBOT VARIABLES 
%robot_names = receive(active_robots,100);
%robot_names = robot_names.Data

robot_names = "HexaTilted";
robot_names = split(robot_names);

robot_number = length(robot_names);

robot_topics = dictionary; 
wheel_publishers = dictionary; 

dx = zeros(robot_number);
dy = zeros(robot_number);
dx(1) = 0;
dy(1) = 0;
dtheta = zeros(robot_number);

wheel_radius = 0.05;
robot_l = 0.10;
robot_w = 0.10;

matrix = 1/wheel_radius*[1 -1 -(robot_l+robot_w); 1 1 (robot_l+robot_w); 1 -1 (robot_l+robot_w); 1 1 -(robot_l+robot_w)];


%%%% GIMBAL VARIABLES4906586
xpd = zeros(robot_number);
xyd = zeros(robot_number);

max_gimbal_speed = 180;


%%%% SPEED CONSTANTS 
max_linear = 0.9;
max_theta = 2.4906586;

for i = 1:robot_number
    sub_topic = "/vicon/"+robot_names(i)+"/"+robot_names(i);
    subscriber = rossubscriber(sub_topic,"geometry_msgs/TransformStamped");
    robot_topics(robot_names(i)) = subscriber;   
    disp(robot_topics("HexaTilted"))
    pause(1)


    wheel_publishers(robot_names(i)) = rospublisher(string(robot_names(i))+'/wheels',"geometry_msgs/Quaternion");
    pause(0.5)

end

[axes, buttons, ~] = read(joy);

while buttons(1,2) == 0

    robot_speeds = '';

    gimbal_speeds = '';
    %gimbal_data = receive(gimbal_state,10);
    %gimbal_data = split(gimbal_data.Data,'~');

    for i = 1:robot_number

        %%%%% ROBOT CONTROL
        disp('receiving data')
        data = receive(robot_topics("HexaTilted"),10);
        

        robot_pose = getPose(data);
        robot_pose
       
        ux = speed_controller(robot_pose(1),dx(robot_number),max_linear);
        uy = speed_controller(robot_pose(2),dy(robot_number),max_linear);
        uw = speed_controller(robot_pose(4),dtheta(robot_number),max_theta);

        speed_vec = [-uy;-ux;uw];

        wheelSpeed = matrix*speed_vec;

        wheelSpeed = wheelSpeed*(30/pi);

        temp = transpose(wheelSpeed)

        send_twist(wheel_publishers(robot_names(i)),temp)
           
        %robot_speeds = format_string(robot_speeds,temp,4);
        

        % %%%%% GIMBAL CONTROL
        % 
        % [axes, buttons, ~] = read(joy);
        % 
        % g_speed = split(gimbal_data(i),',');
        % xp = str2double(g_speed(1));
        % xy = str2double(g_speed(2));
        % up = control_calc(xpd(i),xp,max_gimbal_speed);
        % uy = control_calc(xyd(i),xy,max_gimbal_speed);
        % temp = [up,uy];
        % 
        % gimbal_speeds = format_string(gimbal_speeds,temp,2);

    end
    
    %%%%%%%%%%%%%%%%%%%%%%% ENVIA INFORMACION 
    %%%% ENVIA ROBOTS
    %send_ros(pub_wheel_speed,robot_speeds)

    % %%%% ENVIA GIMBAL
    % send_ros(pub_gimbal_speed,gimbal_speeds)


end  

send_ros(state_publisher,'shutdown')
rosshutdown

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

function u = speed_controller(c,d,k)
    e = (c - d)/k;
    %disp(-200*tanh(e))
    u = k*tanh(e);

end

function base_string = format_string(base_string,data,string_size)
    for j = 1:string_size
        base_string = strcat(base_string,num2str(data(j)));
        if j < string_size
            base_string = strcat(base_string,',');
        else
            base_string = strcat(base_string,'~');
        end
    end

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