
clear; clc

% rosshutdown 
% rosinit 
% setenv('ROS_MASTER_URI', 'http://192.168.0.145:11311')

% sigmanoise = 0.000000;
% Obs = InitObserver();

% tf = Inf;
% mdl = "send_pwm.slx";
% open_system(mdl)

% mdl = 'vicon_hexatilted_read';
% open_system(mdl)

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

function Observer = InitObserver()
    alpha = [0.5; 0.5; 0.5; 0.5; 0.5; 0.5; 0.5; 0.5];
    k1 =    [0.01  ; 0.01  ; 0.001  ; 0.01  ; 0.01  ; 0.01  ; 0.2  ; 0.2  ];
    k2 =    [0.01  ; 0.01  ; 0.01  ; 0.01  ; 0.01  ; 0.01  ; 0.01  ; 0.01  ];
    x10   = [0  ; 0  ; 1  ; 0  ; 0  ; 0  ; 0  ; 0  ];
    x20   = [0  ; 0  ; 0  ; 0  ; 0  ; 0  ; 0  ; 0  ];
    Observer = struct('a' , alpha,...
                      'k1', k1,...
                      'k2', k2,...
                      'x10', x10,...
                      'x20', x20);
end
