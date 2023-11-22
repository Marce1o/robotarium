%dbstop if error
clear; clc

% rosshutdown 
% close_system('vicon_hexatilted_read')
% rosinit 

sigmanoise = 0.000000;

Obs = InitObserver();

tf = 5;
open_system('vicon_hexatilted_read')
sim("vicon_hexatilted_read.slx")

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
