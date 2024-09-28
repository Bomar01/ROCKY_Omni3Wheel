%%%%%%%%%%% Conexión a ROCKY a través de la Jetson Nano %%%%%%%%%%%

%% Comando para conexion de la Jetson 

% setenv('ROS_IP','192.168.43.190')          %IP de la computadora
% rosinit('http://192.168.43.178:11311');    %Conexión al nodo maestro   
% rostopic list                              %Comando para revisar los topicos
%% Inicializacion de los topicos
%Topico de punto deseado
msg = rosmessage('std_msgs/Float32MultiArray');
% Publicar el mensaje
robotPD= rospublisher('/punto_deseado', 'std_msgs/Float32MultiArray');
%Topico de la odometria
odomSub = rossubscriber('/odom', 'nav_msgs/Odometry');
%Lectura del tiempo de la Jetson
startTime=rostime('now');
startTime=startTime.Sec
currentTime=rostime('now');
currentTime=currentTime.Sec
limite=rostime('now');
limite=limite.Sec;
restart=rostime('now');
restart=restart.Sec;
%% Inicializacion de variables
%Puntos obtenidos del algoritmo PRM 
 prm_x=puntosx
 prm_y=puntosy
 n=length(prm_x)
%Variables para guardar los datos
vectorx=[];
vectory=[];
vectorxh=[];
vectoryh=[];
vectortetha=[];
i=1;
j=0
x=0
y=0
%% Ejecución del código durante 200 segundos
while  (abs(prm_x(n)-x)>0.1 || abs(prm_y(n)-y)>0.1)
        %ObtenciÓn  de la odometría de ROCKY
        odomMsg=receive(odomSub,5);
        pose=odomMsg.Pose.Pose;
        x=pose.Position.X
        y=pose.Position.Y
        z=pose.Position.Z;
             
        %Posición en X e Y
        vectorx=[vectorx,x];
        vectory=[vectory,y];

        if (j<n)
            j=j+1
           
            data=[prm_x(j),prm_y(j)]
            msg.Data = single(data);
            send(robotPD, msg);
             pause(10)

        end
        
            i=i+1
end
    
%Obtención de la trayectoria
plot(vectorx(1:i-1),vectory(1:i-1),'r','LineWidth',2)
xlim([0,2])
ylim([-0.2, 0.4])
title('Posición')
set(gca, 'FontName','Times New Roman','FontSize', 14,  'FontAngle', 'italic')
set(gcf,'color','w')
xlabel('Posición en x (m)')
ylabel('Posición en y (m)')
 
