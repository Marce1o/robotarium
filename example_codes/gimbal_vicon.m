clc 
clear all
rosshutdown
rosinit

subscriber = rossubscriber("/vicon/gimbal_rm_3/gimbal_rm_3","geometry_msgs/TransformStamped");
pause(1)

while 1
data = receive(subscriber,10);

pose_x = data.Transform.Translation.X;
pose_y = data.Transform.Translation.Y;
pose_z = data.Transform.Translation.Z;

angle_x = data.Transform.Rotation.X;
angle_y = data.Transform.Rotation.Y;
angle_z = data.Transform.Rotation.Z;
angle_w = data.Transform.Rotation.W;

[yaw,pitch,roll] = quat2angle([angle_x angle_y angle_z angle_w]);
yaw_d = rad2deg(roll)
pitch_d = rad2deg(pitch)
orientation = [yaw_d,pitch_d];

end