close all;
clear all;

%%Cierra e inicio de cliente de ROS
rosshutdown
rosinit

%%Configuracion del publicador de ROS con el nombre control del tipo TWIST

wheel_speed = rospublisher("/robots_data","std_msgs/String","DataFormat","struct");
state_publisher = rospublisher("/state","std_msgs/String","DataFormat","struct");

%%Configuracion de control remoto 
ID = 1;
joy=vrjoystick(ID);

%Frecuencia de actualizaciono
updatefreq = 30;
dt = 1/updatefreq;
wt = 1;

%Movement Control 

max_rpm = 100;

n_robots = 4; 



while 1
    [axes, buttons, ~] = read(joy);
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% WAITING MODE
    if buttons(1,2) == 1
        pub_msg = rosmessage(state_publisher);
        pub_msg.Data = 'shutdown';
        send(state_publisher,pub_msg)
        pause(dt)
        break
    end

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% MOVEMENT MODE
        %Obtencion de valores del control de xbox 
    speeds='';
    for i = 1:n_robots
        if rem(i,2) == 0 
            temp = move_bot(axes(1,2),axes(1,1),max_rpm);
        elseif rem(i,2) ~= 0
            temp = move_bot(axes(1,5),axes(1,4),max_rpm);
        end
        for j = 1:4
            speeds = strcat(speeds,num2str(temp(j)));
            if j ~= 4
                speeds = strcat(speeds,',');
            else
                if i ~=n_robots
                    speeds = strcat(speeds,'~');
                end
            end
        end
 
    end 
   
    pub_msg = rosmessage(wheel_speed);
       
    %Configuramos que se va a mandar
    speeds
    %break
    %speeds = cell2mat(speeds);

    disp(speeds)
    
    pub_msg.Data = speeds;
    %pub_msg.Data = '[0,0,0,0],[0,0,0,0],[0,0,0,0],[0,0,0,0]';
    send(wheel_speed,pub_msg)

    pause(dt)
    
   
end


function wheels = move_bot(x,y,max_rpm)
    disp("Moving Robot")
    wheels = [];
    if abs(x) > 0.05 || abs(y) > 0.05
        if abs(x) > abs(y)
            vel = -x*max_rpm;
            wheels = [vel,vel,vel,vel];
        elseif abs(x) < abs(y)
            vel = y*max_rpm;
            if y > 0
                wheels = [-vel,vel,vel,-vel];
            elseif y < 0
                wheels = [-vel,vel,vel,-vel];
            end
        end
    else 
        wheels = [0,0,0,0];
    end
         
    disp(wheels)
end
