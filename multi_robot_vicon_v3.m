dbstop if error
clc
close all
clear all 

rosshutdown 

rosinit 

ID = 1;
joy=vrjoystick(ID);


%%%% ROS PUBLISHERS 
pub_wheel_speed = rospublisher("/robots_data","std_msgs/String","DataFormat","struct");
pub_gimbal_speed = rospublisher("/blaster","std_msgs/String","DataFormat","struct");
state_publisher = rospublisher("/state","std_msgs/String","DataFormat","struct");


%%%% ROS SUBSCRIBERS 
active_robots = rossubscriber("/robots","std_msgs/String","DataFormat","struct");
pause(1)
gimbal_state = rossubscriber("/gimbal_state","std_msgs/String","DataFormat","struct");
pause(1)


%%%% ROBOT VARIABLES 
%robot_names = receive(active_robots,100);
%robot_names = robot_names.Data;

robot_names = "rm_5 gimball_5";
robot_names = split(robot_names);

robot_number = length(robot_names);

robot_topics = dictionary; 
dx = zeros(robot_number);
dy = zeros(robot_number);
dtheta = zeros(robot_number);

wheel_radius = 0.05;
robot_l = 0.10;
robot_w = 0.10;

matrix = 1/wheel_radius*[1 -1 -(robot_l+robot_w); 1 1 (robot_l+robot_w); 1 -1 (robot_l+robot_w); 1 1 -(robot_l+robot_w)];


%%%% GIMBAL VARIABLES4906586
xpd = zeros(num_robots);
xyd = zeros(num_robots);

max_gimbal_speed = 180;


%%%% SPEED CONSTANTS 
max_linear = 1.5;
max_theta = 3.4906586;

for i = 1:robot_number
    sub_topic = "/vicon/"+robot_names(i)+"/"+robot_names(i);
    subscriber = rossubscriber(sub_topic,"geometry_msgs/TransformStamped");
    robot_topics(robot_names(i)) = subscriber;    
    pause(1)
    
end

[axes, buttons, ~] = read(joy);

while buttons(1,2) == 0

    robot_speeds = '';

    gimbal_speeds = '';
    gimbal_data = receive(gimbal_state,10);
    gimbal_data = split(gimbal_data.Data,'~');

    for i = 1:robot_number

        %%%%% ROBOT CONTROL
        data = receive(robot_topics(robot_names(i)),10);


        robot_pose = getPose(data);
       
        ux = speed_controller(robot_pose(1),dx(robot_number),max_linear);
        uy = speed_controller(robot_pose(2),dy(robot_number),max_linear);
        uw = speed_controller(robot_pose(4),dtheta(robot_number),max_theta);

        speed_vec = [ux;uy;uw];

        wheelSpeed = matrix*speed_vec;

        wheelSpeed = wheelSpeed*(30/pi);

        temp = transpose(wheelSpeed);
           
        robot_speeds = format_string(robot_speeds,temp,4);
        

        %%%%% GIMBAL CONTROL

        [axes, buttons, ~] = read(joy);

        g_speed = split(gimbal_data(i),',');
        xp = str2double(g_speed(1));
        xy = str2double(g_speed(2));
        up = control_calc(xpd(i),xp,max_gimbal_speed);
        uy = control_calc(xyd(i),xy,max_gimbal_speed);
        temp = [up,uy];

        gimbal_speeds = format_string(gimbal_speeds,temp,2);

    end
    
    %%%%%%%%%%%%%%%%%%%%%%% ENVIA INFORMACION 
    %%%% ENVIA ROBOTS
    send_ros(pub_wheel_speed,robot_speeds)

    %%%% ENVIA GIMBAL
    send_ros(pub_gimbal_speed,gimbal_speeds)


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
    u = -k*tanh(e);

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