function stop(N,config)

    for i =  1:N.RobomasterF
        send_twist(config.w_pub(config.robot_names(i)),[0,0,0,0])
        send_point(config.g_pub(config.robot_names(i)),[0,0])
    end 

    send_ros(config.state_pub,'shutdown')
    rosshutdown
end

function send_ros(topic,data)
    pub_msg = rosmessage(topic);
    pub_msg.Data = data;
    send(topic,pub_msg)
    
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