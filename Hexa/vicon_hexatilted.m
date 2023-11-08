%dbstop if error
clear, clc

% rosshutdown 
% 
% rosinit 

sigmanoise = 0.00000005;

tf = 3;
open_system('vicon_hexatilted_read')
% sim("vicon_hexatilted_read.slx")

% rosshutdown
