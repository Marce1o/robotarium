%dbstop if error
clear, clc

rosshutdown 
close_system('vicon_hexatilted_read')
rosinit 

sigmanoise = 0.0000001;

tf = Inf;
open_system('vicon_hexatilted_read')
sim("vicon_hexatilted_read.slx")
