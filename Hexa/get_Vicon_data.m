
function [pos, BF_att] = get_Vicon_data(obj)

    robot_number = obj.Number;
    robot_names = obj.Names;
    robot_topic = obj.Topics;
    
    for i = 1:robot_number
    
            %%%%% ROBOT CONTROL
            disp('receiving data: ' + robot_names(i))
            data = receive(robot_topic,10);
            
            [pos, BF_att] = getPose(data);
            
    end

end


function [pos, BF_att] = getPose(data)

        pos = zeros(3,1);
        pos(1) = data.Transform.Translation.X;
        pos(2) = data.Transform.Translation.Y;
        pos(3) = data.Transform.Translation.Z;

        angle_x = data.Transform.Rotation.X;
        angle_y = data.Transform.Rotation.Y;
        angle_z = data.Transform.Rotation.Z;
        angle_w = data.Transform.Rotation.W;

        [yaw,pitch,roll] = quat2angle([angle_x angle_y angle_z angle_w]);
        BF_att = [yaw, pitch, roll];

end 

