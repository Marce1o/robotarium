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
blaster_speed = rospublisher("/blaster","std_msgs/String","DataFormat","struct");
state_publisher = rospublisher("/state","std_msgs/String","DataFormat","struct");


%%%% ROS SUBSCRIBERS 
active_robots = rossubscriber("/robots","std_msgs/String","DataFormat","struct");
pause(1)
gimbal_state = rossubscriber("/gimbal_state","std_msgs/String","DataFormat","struct");
pause(1)


%%%% ROBOT VARIABLES 
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


%%%% GIMBAL VARIABLES



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

    speeds = '';
    for i = 1:robot_number
        data = receive(robot_topics(robot_names(i)),10);

        %robot_poses(robot_names(i)) = getPose(data)

        robot_pose = getPose(data);
       
        ux = speed_controller(robot_pose(1),dx(robot_number),max_linear);
        uy = speed_controller(robot_pose(2),dy(robot_number),max_linear);
        uw = speed_controller(robot_pose(4),dtheta(robot_number),max_theta);

        speed_vec = [ux;uy;uw];

        wheelSpeed = matrix*speed_vec;

        wheelSpeed = wheelSpeed*(30/pi);

        temp = transpose(wheelSpeed);
           
        speeds = format_string(speeds,temp,4);

        [axes, buttons, ~] = read(joy);

    end
    
    send_ros(pub_wheel_speed,speeds)


end  

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
        if j < 4
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