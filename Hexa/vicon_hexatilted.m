%dbstop if error
clear, clc

% rosshutdown 
% close_system('vicon_hexatilted_read')
% rosinit 
% setenv('ROS_MASTER_URI', 'http://192.168.0.145:11311')

tf = Inf;
mdl = "send_pwm.slx";
open_system(mdl)
% sim(mdl)

%% Before connecting ESP

% rosnode list
% /matlab_global_node_27250
% /rosout
% /send_pwm_90580
% /vicon

% rostopic list
% /PWM                  
% /diagnostics          
% /rosout               
% /rosout_agg           
% /tf                   
% /vicon/QuadGus/QuadGus
% /vicon/markers    


