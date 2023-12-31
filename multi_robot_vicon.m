close all;
clear all;

%%Cierra e inicio de cliente de ROS
rosshutdown
rosinit

%%Configuracion del publicador de ROS con sus tipos correspondientes

wheel_speed = rospublisher("/robots_data","std_msgs/String","DataFormat","struct");
blaster_speed = rospublisher("/blaster","std_msgs/String","DataFormat","struct");
state_publisher = rospublisher("/state","std_msgs/String","DataFormat","struct");



active_robots = rossubscriber("/robots","std_msgs/String","DataFormat","struct");
gimbal_state = rossubscriber("/gimbal_state","std_msgs/String","DataFormat","struct");


%n_robots = receive(active_robots,100);
%n_robots = receive(active_robots,100);
%n_robots = n_robots.Data;
%n_robots = split(n_robots,',');

n_robots = 'rm_5';
n_robots = split(n_robots,',');

num_robots = length(n_robots);

pose_suscribers = dictionary;
robots_suscribers = dictionary;
%robots_suscribers = configureDictionary("string","ros.Subscriber")

for i = 1:num_robots
    %disp(i)
    %disp(n_robots(i))
    tosusc = "/vicon/"+n_robots(i)+'/'+n_robots(i);
    disp(tosusc)
    suscribe = rossubscriber(tosusc,"geometry_msgs/TransformStamped","DataFormat","struct");
    robots_suscribers(n_robots(i)) = suscribe;
    %disp(robots_suscribers)
end

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
robotSelector = 1;

max_speed = 150;

xpd = zeros(num_robots);
xyd = zeros(num_robots);

summed_p = 0;
summed_y = 0; 

%%%%%%%%%%%%%%% CONSTANTS 

k = 0.72;
gimbal_vmax = 180;


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
       

    for i = 1:num_robots
        disp(robots_suscribers)
        disp(n_robots(i));
        data = receive(suscribe);
        pose = data.Transform.Translation;
        pose_x = data.Transform.Translation.X;
        pose_y = data.Transform.Translation.Y;
        pose_z = data.Transform.Translation.Z;

        %disp(pose_x)
        %disp(pose_y)
        %disp(pose_z)
        disp(pose)
        
    end

    %%%%%%%%%%%%%%%%%%%%%%%% Alternate robots
    if buttons(1,5) == 0 && buttons(1,6) == 0
        lastNext = 0;
        lastPrev = 0;
    end
    
    if buttons(1,6) == 1 && lastNext ~= 1
        if robotSelector < num_robots
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
    
    %%%%%%%%%% CONTROLADOR
    if axes(1,8) == -1 && summed_p ~= 1 && xpd(robotSelector) < 30
        xpd(robotSelector) = xpd(robotSelector) + 5;
        summed_p = 1;

    elseif axes(1,8) == 1 && summed_p ~= 1 && xpd(robotSelector) > -25
        xpd(robotSelector) = xpd(robotSelector) - 5;
        summed_p = 1;
    else
        summed_p = 0;
    end 

    if axes(1,7) == 1 && summed_y ~= 1 && xyd(robotSelector) < 250
        xyd(robotSelector) = xyd(robotSelector) + 10;
        summed_y = 1;
    elseif axes(1,7) == -1 && summed_y ~= 1 && xyd(robotSelector) > -250
        xyd(robotSelector) = xyd(robotSelector) - 10;
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
    for i = 1:num_robots
        temp = move_bot(axes(1,5),axes(1,4),max_rpm);
        if(robotSelector ~= i)
            temp = [0,0,0,0];
        end
        for j = 1:4
            speeds = strcat(speeds,num2str(temp(j)));
            if j ~= 4
                speeds = strcat(speeds,',');
            else
                if i ~= num_robots
                    speeds = strcat(speeds,'~');
                end
            end
        end
    end 

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% ROS SEND INFO
    pub_msg = rosmessage(wheel_speed);

    %disp(speeds)
    pub_msg.Data = speeds;
    %pub_msg.Data = '[0,0,0,0],[0,0,0,0],[0,0,0,0],[0,0,0,0]';
    send(wheel_speed,pub_msg)

    speed_blaster = '';

    % try 
    %     gimbal_data = receive(gimbal_state,100);
    % 
    %     gimbal_data = split(gimbal_data.Data,'~');
    % 
    %     for i = 1:num_robots
    %         speeds = split(gimbal_data(i),',');
    %         xp = str2double(speeds(1));
    %         xy = str2double(speeds(2));
    %         up = control_calc(xpd(i),xp,gimbal_vmax);
    %         uy = control_calc(xyd(i),xy,gimbal_vmax);
    %         disp(xpd)
    %         disp(xyd)
    %         temp = [up,uy];
    % 
    %         if(robotSelector ~= i)
    %             temp = [0,0];
    %         end
    %         for j = 1:2
    %             speed_blaster = strcat(speed_blaster,num2str(temp(j)));
    %             if j ~= 2
    %                 speed_blaster = strcat(speed_blaster,',');
    %             else
    %                 if i ~= n_robots
    %                     speed_blaster = strcat(speed_blaster,'~');
    %                 end
    %             end
    %         end
    %     end 
    % catch 
    %     disp("error")
    % end
    % 
    % 
    % 
    % %disp('next blaster')
    % 
    % %disp(speed_blaster)
    % pub_msg = rosmessage(blaster_speed);
    % pub_msg.Data = speed_blaster;
    % send(blaster_speed,pub_msg)
    % 
    % 
    % pause(dt)    
   
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
    %u = -0.72*(x-xd);
    
    e = (x - xd)/180
    u = -k*tanh(e)

end 


function send_ros(topic,data)
    pub_msg = rosmessage(topic);
    pub_msg.Data = data;
    send(topic,pub_msg)
    
end
