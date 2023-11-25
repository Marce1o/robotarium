function [config,Nmax] = init()

    rosshutdown
    rosinit

    config.state_pub = rospublisher("/exec_state","std_msgs/String","DataFormat","struct");
    config.active_robots = rossubscriber("/robot_names","std_msgs/String","DataFormat","struct");
    ID = 1;
    config.joy=vrjoystick(ID);

    config.r_sub = dictionary;
    config.g_sub = dictionary;
    config.w_pub = dictionary;
    config.g_pub = dictionary;

    %%%% ROBOT VARIABLES 
    robot_names = receive(config.active_robots,100);
    config.robot_names = robot_names.Data;
    config.robot_names = split(config.robot_names);
    
    Nmax.RobomasterF = length(config.robot_names);

    
   for i = 1:N.RobomasterFMax
    config.r_sub(config.robot_names(i)) = rossubscriber("/vicon/"+config.robot_names(i)+"/"+config.robot_names(i),"geometry_msgs/TransformStamped");  
    pause(1)

    config.g_sub(config.robot_names(i)) = rossubscriber("/vicon/gimbal_"+config.robot_names(i)+"/gimbal_"+config.robot_names(i),"geometry_msgs/TransformStamped");
    pause(1)

    config.w_pub(config.robot_names(i)) = rospublisher(string(config.robot_names(i))+'/wheels',"geometry_msgs/Quaternion");
    pause(0.5)

    config.g_pub(config.robot_names(i)) = rospublisher(string(config.robot_names(i))+'/gimbal_speed',"geometry_msgs/Point");
    pause(0.5)
   end

end