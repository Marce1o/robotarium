close all;
clear all;

%%Cierra e inicio de cliente de ROS
rosshutdown
rosinit

%%Configuracion del publicador de ROS con el nombre control del tipo TWIST

r1_publisher = rospublisher("/robot_1","geometry_msgs/Twist","DataFormat","struct");
r2_publisher = rospublisher("/robot_2","geometry_msgs/Twist","DataFormat","struct");
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

    move_bot(axes(1,2),axes(1,1),r1_publisher,max_rpm)
    move_bot(axes(1,5),axes(1,4),r2_publisher,max_rpm)

   
    pause(dt)
    
   
end


function move_bot(x,y,topic,max_rpm)
    disp("Moving Robot")
    wheels = {0,0,0,0};
    if abs(x) > abs(y)
        vel = -x*max_rpm;
        wheels = {vel,vel,vel,vel};
    elseif abs(x) < abs(y)
        vel = y*max_rpm;
        if y > 0
            wheels = {-vel,vel,vel,-vel};
        elseif y < 0
            wheels = {-vel,vel,vel,-vel};
        end
       
    end

    pub_msg = rosmessage(topic);
       
    %Configuramos que se va a mandar
    pub_msg.Linear.X = wheels{1};
    pub_msg.Linear.Y = wheels{2};
    pub_msg.Linear.Z = wheels{3};
    pub_msg.Angular.X = wheels{4};
    send(topic,pub_msg)
    %Pausa para la frecuencia del robot 
   
end
