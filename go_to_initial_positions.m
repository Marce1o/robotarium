function go_to_initial_positions(N,initial_conditions,config)
            w_rad = 0.05;
            L = 0.1;
            W = 0.1;
            M = [1, 1, (L+W);...
                 1,-1,-(L+W);...
                 1, 1,-(L+W);...
                 1,-1, (L+W)];

            k_linear = 0.5;
            k_theta = 2.4906586;
            k_gimbal = 90; %90 deg/s max speed
        
        state       = zeros(5,N.RobomasterF); 
        stated      = zeros(5,N.RobomasterF);
        error       = ones(5,N.RobomasterF);
        error_large = ones(N.RobomasterF);
        
        while norm(error_large)>0            
            for i = 1:N.RobomasterF
                stated(:,i)= [initial_conditions.RobomasterF(1,i);...
                              initial_conditions.RobomasterF(2,i);...
                              initial_conditions.RobomasterF(3,i);...
                              initial_conditions.RobomasterF(4,i);...
                              initial_conditions.RobomasterF(5,i)];
    
                state(:,i) = get_Pose(config,i);
    
                error(:,i) = state(:,i) - stated(:,i);
                    
                if norm(error(:,i)) >= 0.01
                    BRW = [ cos(state(3,i)),sin(state(3,i)),0;...
                       -sin(state(3,i)),cos(state(3,i)),0;...
                             0    ,   0    ,1];
                    wheel_speed = (30/pi)*(1/w_rad)*M*BRW*[-k_linear*tanh(2*(error(1,i)));...
                                                           -k_linear*tanh(2*(error(2,i)));...
                                                           -k_theta*tanh(2*(error(3,i)))];
        
                    gimbal_speed = [k_gimbal*tanh(2*(error(4,i)));...
                                    k_gimbal*tanh(2*(error(5,i)))];
        
                    send_twist(config.w_pub(config.robot_names(i)),wheel_speed)
                    send_point(config.g_pub(config.robot_names(i)),gimbal_speed)      
                    error_large(i) = 1;
                else
                    send_twist(config.w_pub(config.robot_names(i)),[0,0,0,0])
                    send_point(config.g_pub(config.robot_names(i)),[0,0,0,0])
                    error_large(i) = 0;
                end
            end
        end
  
end

function [x,y,th,yaw,pitch] = get_Pose(config,iteration)
        pose_data = receive(config.r_sub(config.robot_names(iteration)),10);
        gimbal_data = receive(config.g_sub(config.robot_names(iteration)),10);

        robot_pose = getInfoPose(pose_data);

        x = pose_data(1);
        y = pose_data(2);
        th = pose_data(3);

        gimbal_orientation = getOrientation(gimbal_data);
        gimbal_orientation(1) = gimbal_orientation(1) - rad2deg(robot_pose(3));

        yaw = gimbal_orientation(1);
        pitch = gimbal_orientation(2);

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
        orientation = [yaw,pitch]; % radians!!
end

function send_twist(topic,data)
    msg = rosmessage(topic);
    msg.X = data(1);
    msg.Y = data(2);
    msg.Z = data(3);
    msg.W = data(4);

    send(topic,msg)
end


function send_point(topic,data)
    msg = rosmessage(topic);
    msg.X = data(1);
    msg.Y = data(2);

    send(topic,msg)
end