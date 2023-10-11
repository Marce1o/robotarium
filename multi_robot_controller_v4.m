close all;
clear all;

%%Cierra e inicio de cliente de ROS
rosshutdown
rosinit

%%Configuracion del publicador de ROS con sus tipos correspondientes

wheel_speed = rospublisher("/robots_data","std_msgs/String","DataFormat","struct");
blaster_speed = rospublisher("/blaster","std_msgs/String","DataFormat","struct");
state_publisher = rospublisher("/state","std_msgs/String","DataFormat","struct");
active_robots = rossubscriber("/robots","std_msgs/Int16","DataFormat","struct");
gimbal_state = rossubscriber("/gimbal_state","std_msgs/String","DataFormat","struct");

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

max_speed = 150;

n_robots = receive(active_robots,100);
n_robots = n_robots.Data;

xpd = zeros(n_robots);
xyd = zeros(n_robots);

summed_p = 0;
summed_y = 0; 

%%%%%%%%%%%%%%% CONSTANTS 

k = 0.72;




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

    if axes(1,8) == -1 && summed_p ~= 1 && xpd(robotSelector) < 250
        xpd(robotSelector) = xpd(robotSelector) + 10;
        summed_p = 1;

    elseif axes(1,8) == 1 && summed_p ~= 1 && xpd(robotSelector) > -250
        xpd(robotSelector) = xpd(robotSelector) - 10;
        summed_p = 1;
    else
        summed_p = 0;
    end 

    if axes(1,7) == 1 && summed_y ~= 1 && xyd(robotSelector) < 30
        xyd(robotSelector) = xyd(robotSelector) + 5;
        summed_y = 1;
    elseif axes(1,7) == -1 && summed_y ~= 1 && xyd(robotSelector) > -25
        xyd(robotSelector) = xyd(robotSelector) - 5;
        summed_y = 1;
    else
        summed_y = 0;
    end 

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Move desired blaster position 

    
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

    speed_blaster = '';

    %try 
        gimbal_data = receive(gimbal_state,100);

        gimbal_data = split(gimbal_data.Data,'~');
        
        for i = 1:n_robots
            speeds = split(gimbal_data(i),',');
            xp = str2double(speeds(1));
            xy = str2double(speeds(2));
            up = control_calc(xpd(i),xp,k);
            uy = control_calc(xyd(i),xy,k);
            temp = [up,uy];
            
            if(robotSelector ~= i)
                temp = [0,0];
            end
            for j = 1:2
                speed_blaster = strcat(speed_blaster,num2str(temp(j)));
                if j ~= 2
                    speed_blaster = strcat(speed_blaster,',');
                else
                    if i ~= n_robots
                        speed_blaster = strcat(speed_blaster,'~');
                    end
                end
            end
        end 
    %catch 
    %    disp("error")
    %end

   

    disp('next blaster')

    disp(speed_blaster)
    pub_msg = rosmessage(blaster_speed);
    pub_msg.Data = speed_blaster;
    send(blaster_speed,pub_msg)
    

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


function blaster = move_blaster(x,y,max_speed)
    blaster = [];
    vel_p = 0;
    vel_y = 0;
    if abs(x) > 0.05 || abs(y) > 0.05
        if abs(x) > abs(y)
            vel_p = -x*max_speed;
            vel_y = 0;
        elseif abs(x) < abs(y)
            vel_y = y*max_speed;
            vel_p = 0;
        end

        blaster = [vel_p,vel_y];
    else 
        blaster = [vel_p,vel_y];
    end
end

function u = control_calc(xd,x,k)
    u = -k*(x-xd)
end 


function send_ros(topic,data)
    pub_msg = rosmessage(topic);
    pub_msg.Data = data;
    send(topic,pub_msg)
    
end
