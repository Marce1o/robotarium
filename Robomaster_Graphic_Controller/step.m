function r = step(r,w,N,config)
    if N.RobomasterF>0
        r = step_RobomasterF(r,w,N,config);
        display_RobomasterF(r,N);
    end

    %Reset checks for get_pose
    r.checked_poses_already = false;
    r.called_step_already = true;
end

function r = step_RobomasterF(r,w,N,config)

    for i=1:1:N.RobomasterF
        send_twist(config.w_pub(config.robot_names(i)),w.RobomasterF(:,i))
        send_point(config.g_pub(config.robot_names(i)),g.RobomasterF(:,i))
    end
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