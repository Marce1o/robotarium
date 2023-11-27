function r = start_arena(N,initial_conditions,config)

    go_to_initial_positions(N,initial_conditions,config)
    
    %%Simulation properties%%
    r.Dt = 0.03; %Iteration time
    
    r.checked_poses_already = false; % Whether GET_POSES has been checked this iteration
    r.called_step_already = true; % Whether STEP has been called this iteration
    r.iteration = 0; % How many times STEP has been called
    %errors = {}; % Accumulated errors for the simulation

    %%Arena Properties%%

    %arenavertices = [x_i,y_i,z_i]
    r.arena_vertices = [ 6.5 ,-2.5,0;...
                         6.5 , 2.5,0;...
                        -6.5 , 2.5,0;...
                        -6.5 , 0.5,0;...
                        -3.14,-2.5,0];
    %arean_boundaries = [xmin, xmax, ymin, ymax]
    arena_boundaries = [min(r.arena_vertices(:,1)), max(r.arena_vertices(:,1)), min(r.arena_vertices(:,2)), max(r.arena_vertices(:,2))];
    offset = 0.05;

    %%Display arena%%
    
    %Figuer handle
    r.figure_handle = figure;
    fig = r.figure_handle;    
    %Boundary patch
    r.boundary_patch = patch('XData', [r.arena_vertices(:,1);r.arena_vertices(1,1)], ...
                             'YData', [r.arena_vertices(:,2);r.arena_vertices(1,2)], ...
                             'FaceColor', 'none', ...
                             'LineWidth', 3, ...,
                             'EdgeColor', [0, 0, 0]);
    % Set axis
    ax = fig.CurrentAxes;
    % Limit view to xMin/xMax/yMin/yMax
    axis(ax, [arena_boundaries(1)-offset, arena_boundaries(2)+offset, arena_boundaries(3)-offset, arena_boundaries(4)+offset])
    set(ax, 'PlotBoxAspectRatio', [1 1 1], 'DataAspectRatio', [1 1 1])
    axis(ax, 'off')            
    hold on

    %%Display robots%%

    assert(N.RobomasterF >= 0 && N.RobomasterF <= 5, 'Number of RobomasterF (%i) must be >= 0 and <= 5', N.RobomasterF);
    
    r.poses.RobomasterF = zeros(3,N.RobomasterF);
    r.ptu.RobomasterF = zeros(2,N.RobomasterF);

    %RobomasterF
    %r.robot_handle.robomasterF = cell(1, N.robomasterF);
    data = RobomasterF_patch;
    %vertices in ROWS to use matlab's patch vertex-face format
    base_robot_vertices = data.vertices;
    for i=1:1:N.RobomasterF
        r.poses.RobomasterF(:,i) = initial_conditions.RobomasterF(1:3,i);
        %r.ptu.robomasterF(:,i)   = intial_conditions.RobomasterF(4:5,i);
        x  = r.poses.RobomasterF(1,i);
        y  = r.poses.RobomasterF(2,i);
        th = r.poses.RobomasterF(3,i);
        %Not, WRB, but WTB because we are using homogeneous coords in 2D!!
        WTB = [cos(th), -sin(th), x;
               sin(th),  cos(th), y;
                  0   ,     0   , 1]; 
        current_robot_vertices = (base_robot_vertices*WTB');
        r.patch_handle.RobomasterF(i) = patch('Vertices', current_robot_vertices(:, 1:2), ...
                                      'Faces', data.faces, ...
                                      'FaceColor', 'flat', ...
                                      'FaceVertexCData', data.colors, ...
                                      'EdgeColor','none');
    end
end

function go_to_initial_positions(N,initial_conditions,config)
            w_rad = 0.05;
            L = 0.1;
            W = 0.1;
            M = [1, 1, (L+W);...
                 1,-1,-(L+W);...
                 1, 1,-(L+W);...
                 1,-1, (L+W)];

            k_linear = 1;
            k_theta = 2.4906586;
            k_gimbal = 30; %90 deg/s max speed
        
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
                state(4,1)
                stated(4,1)
    
                error(:,i) = state(:,i) - stated(:,i);
                error(4,1)
                    
                if norm(error(:,i)) >= 0.05
                    BRW = [ cos(state(3,i)),sin(state(3,i)),0;...
                       -sin(state(3,i)),cos(state(3,i)),0;...
                             0    ,   0    ,1];
                    wheel_speed = (30/pi)*(1/w_rad)*M*BRW*[-k_linear*tanh((error(1,i)));...
                                                           -k_linear*tanh((error(2,i)));...
                                                           -k_theta*tanh((error(3,i)))];
        
                    %gimbal_speed = [-k_gimbal*(error(4,i));...
                    %                -k_gimbal*(error(5,i))];

                    gimbal_speed = [-k_gimbal*(error(5,i));...
                                    k_gimbal*(error(4,i))];
        
                    send_twist(config.w_pub(config.robot_names(i)),wheel_speed)
                    send_point(config.g_pub(config.robot_names(i)),gimbal_speed)      
                    error_large(i) = 1;
                    error(:,i)
                else
                    send_twist(config.w_pub(config.robot_names(i)),[0,0,0,0])
                    send_point(config.g_pub(config.robot_names(i)),[0,0])
                    error_large(i) = 0;
                end
            end
        end
        disp("In position")
  
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

        yaw = gimbal_orientation(1)
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