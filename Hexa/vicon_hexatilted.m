%dbstop if error
clc
close all
clear all 

rosshutdown 

rosinit 

%%%% ROS PUBLISHERS 
% state_publisher = rospublisher("/state","std_msgs/String","DataFormat","struct");
% 
%%%% ROBOT VARIABLES 
% robot_names = "HexaTilted";
% robot_names = split(robot_names);
% 
% robot_number = length(robot_names);
% 
% robot_topics = dictionary; 
% 
% for i = 1:robot_number
%     sub_topic = "/vicon/"+robot_names(i)+"/"+robot_names(i);
%     subscriber = rossubscriber(sub_topic,"geometry_msgs/TransformStamped");
%     robot_topics(robot_names(i)) = subscriber;   
% end
% 
% obj = struct('Names', robot_names,...
%              'Number', robot_number,...
%              'Topics', robot_topics(robot_names));
% 
% tf = 10;
% 
% for i = 1:10
%     [pos, BF_att] = get_Vicon_data(obj);
%     disp(pos)
% end

tf = Inf;
open_system('vicon_hexatilted_read')
sim("vicon_hexatilted_read.slx")

rosshutdown
