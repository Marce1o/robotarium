clc 
clear all
rosshutdown
rosinit

subscriber = rossubscriber("/vicon/gimbal_rm_3/gimbal_rm_3","geometry_msgs/TransformStamped");
pause(1)
robot_sub = rossubscriber("/vicon/rm_3/rm_3","geometry_msgs/TransformStamped");
pause(1)

while 1
data = receive(subscriber,10);

robot_data = receive(robot_sub,10);

pose_x = data.Transform.Translation.X;
pose_y = data.Transform.Translation.Y;
pose_z = data.Transform.Translation.Z;

angle_x = data.Transform.Rotation.X;
angle_y = data.Transform.Rotation.Y;
angle_z = data.Transform.Rotation.Z;
angle_w = data.Transform.Rotation.W;


robot_x = robot_data.Transform.Rotation.X;
robot_y = robot_data.Transform.Rotation.Y;
robot_z = robot_data.Transform.Rotation.Z;
robot_w = robot_data.Transform.Rotation.W;

[yaw,pitch,roll] = quat2angle([angle_x angle_y angle_z angle_w]);

[yawr,pitchr,rollr] = quat2angle([robot_x robot_y robot_z robot_w]);

roll_d = rad2deg(roll)
roll_d_r = rad2deg(rollr)
comp = atan2(sin(roll - rollr), cos((roll - rollr)))

end