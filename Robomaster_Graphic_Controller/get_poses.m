function [r,poses] = get_poses(r,N,config)
        
        for i = 1:N.RobomasterF
            pose_vector = get_Pose(config,i);
            pose_vector(4) = pose_vector(4) - pose_vector(3);
            poses.RobomasterF(:,i) = pose_vector;
            r.poses.RobomasterF = poses.RobomasterF(:,i);
        end

        display_RobomasterF(r,N)
       
end

function display_RobomasterF(r,N)
    data = RobomasterF_patch;
    %vertices in ROWS to use matlab's patch vertex-face format
    base_robot_vertices = data.vertices;
    for i=1:1:N.RobomasterF
        x  = r.poses.RobomasterF(1,i);
        y  = r.poses.RobomasterF(2,i);
        th = r.poses.RobomasterF(3,i);
        %Not, WRB, but WTB because we are using homogeneous coords in 2D!!
        WTB = [cos(th), -sin(th), x;
               sin(th),  cos(th), y;
                  0   ,     0   , 1]; 
        new_robot_vertices = (base_robot_vertices*WTB');
        set(r.patch_handle.RobomasterF(i), 'Vertices', new_robot_vertices(:, 1:2));
    end
    drawnow limitrate
end

function pose_vector = get_Pose(config,iteration)
        pose_data = receive(config.r_sub(config.robot_names(iteration)),10);
        gimbal_data = receive(config.g_sub(config.robot_names(iteration)),10);

        robot_pose = getInfoPose(pose_data);

        x = robot_pose(1);
        y = robot_pose(2);
        th = robot_pose(3);

        gimbal_orientation = getOrientation(gimbal_data);
        %Ensure that the orientations are in the right range
        gimbal_orientation(1) = atan2(sin(gimbal_orientation(1) - robot_pose(3)), cos(gimbal_orientation(1) - robot_pose(3)));

        yaw = gimbal_orientation(1);
        pitch = gimbal_orientation(2);

        pose_vector = [x;y;th;yaw;pitch];

end

function pose = getInfoPose(data)

        pose_x = data.Transform.Translation.X;
        pose_y = data.Transform.Translation.Y;

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
        %yaw_d = rad2deg(roll); %horizontal
        %pitch_d = rad2deg(pitch); %vertical
        orientation = [roll,pitch]; % radians!!
end