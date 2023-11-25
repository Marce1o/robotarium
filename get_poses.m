function [r,poses] = get_poses(r)
        
        for i = 1:n_robots
            pose_data = receive(r.r_sub(r.r_names(i)),10);
            gimbal_data = receive(r.g_sub(r.r_names(i)),10);
            robot_pose = getPose(pose_data);
            gimbal_orientation = getOrientation(gimbal_data);

            gimbal_orientation(1) = gimbal_orientation(1) - rad2deg(robot_pose(3));
           
            for j = 1:length(robot_pose)
                poses.RobomasterF(j,i) = robot_pose(j);
            end

            for j = 1:length(robot_pose)
                poses.GimbalF(j,i) = gimbal_orientation(j);
            end 
        end
        
    
end

function pose = getPose(data)

        pose_x = data.Transform.Translation.X;
        pose_y = data.Transform.Translation.Y;
        pose_z = data.Transform.Translation.Z;

        angle_x = data.Transform.Rotation.X;
        angle_y = data.Transform.Rotation.Y;
        angle_z = data.Transform.Rotation.Z;
        angle_w = data.Transform.Rotation.W;

        [yaw,pitch,roll] = quat2angle([angle_x angle_y angle_z angle_w]);
        pose = [pose_x pose_y roll];

end 

function orientation = getOrientation(data)

        angle_x = data.Transform.Rotation.X;
        angle_y = data.Transform.Rotation.Y;
        angle_z = data.Transform.Rotation.Z;
        angle_w = data.Transform.Rotation.W;

        [yaw,pitch,roll] = quat2angle([angle_x angle_y angle_z angle_w]);
        yaw_d = rad2deg(roll); %horizontal
        pitch_d = rad2deg(pitch); %vertical
        orientation = [yaw_d,pitch_d];
end