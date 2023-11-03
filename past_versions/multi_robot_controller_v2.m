close all;
clear all;

%%Cierra e inicio de cliente de ROS
rosshutdown
rosinit

%%Configuracion del publicador de ROS con sus tipos correspondientes

wheel_speed = rospublisher("/robots_data","std_msgs/String","DataFormat","struct");
state_publisher = rospublisher("/state","std_msgs/String","DataFormat","struct");
active_robots = rossubscriber("/robots","std_msgs/Int16","DataFormat","struct");

%%Configuracion de control remoto 
ID = 1;
joy=vrjoystick(ID);

%Frecuencia de actualizacion
updatefreq = 30;
dt = 1/updatefreq;

%Declaracion de variables para control de movimiento

max_rpm = 100;
doonce = 0;
lastNext = 0;
lastPrev = 0;
robotSelector = 0;

n_robots = receive(active_robots,100);
n_robots = n_robots.Data;


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


    %%%%%%%%%%%%%%%%%%%%%%%% Alternate robots
    if buttons(1,5) == 0 && buttons(1,6) == 0
        lastNext = 0;
        lastPrev = 0;
    end
    
    if buttons(1,6) == 1 && lastNext ~= 1
        if robotSelector < n_robots
            robotSelector = robotSelector + 1;
            disp(robotSelector)
        end
        lastNext = 1;
        lastPrev = 0;
    end

    if buttons(1,5) == 1 && lastPrev ~= 1
        if robotSelector > 1
            robotSelector = robotSelector - 1;
            disp(robotSelector)
        end
        lastNext = 0;
        lastPrev = 1;
    end
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% MOVEMENT MODE

    if doonce ~= 1
        doonce = doonce + 1;
        disp("Moving Robot")
    end

    speeds='';
    for i = 1:n_robots
        temp = move_bot(axes(1,5),axes(1,4),max_rpm);
        if(robotSelector ~= i)
            temp = [0,0,0,0];
        end
        for j = 1:4
            speeds = strcat(speeds,num2str(temp(j)));
            if j ~= 4
                speeds = strcat(speeds,',');
            else
                if i ~= n_robots
                    speeds = strcat(speeds,'~');
                end
            end
        end
    end 
   

     %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% ROS SEND INFO

    pub_msg = rosmessage(wheel_speed);

    disp(speeds)
    pub_msg.Data = speeds;
    %pub_msg.Data = '[0,0,0,0],[0,0,0,0],[0,0,0,0],[0,0,0,0]';
    send(wheel_speed,pub_msg)

    pause(dt)
    
   
end


% Funcion que asigna los valores para cada una de las ruedas dependiendo de
% los valores de los ejes del joystick
function wheels = move_bot(x,y,max_rpm)
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
         
    %disp(wheels)
end



