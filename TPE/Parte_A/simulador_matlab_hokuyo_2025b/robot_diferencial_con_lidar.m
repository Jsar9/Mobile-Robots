%% Robot diferencial con lidar
% Robotica Movil - 2025 2c
close all
clear all

verMatlab= ver('MATLAB');       % en MATLAB2020a funciona bien, ajustado para R2016b, los demas a pelearla...

simular_ruido_lidar = false;    %simula datos no validos del lidar real, probar si se la banca
use_roomba=true;               % false para desarrollar usando el simulador, true para conectarse al robot real

%% Roomba
if use_roomba   % si se usa el robot real, se inicializa la conexion    
    rosshutdown
    pause(1)
    ipaddress_core = '192.168.0.102';
    ipaddress_local = '192.168.0.105';  %mi ip en a red TurtleNet
    setenv('ROS_IP', '192.168.0.105');
    setenv('ROS_MASTER_URI', ['http://', ipaddress_core, ':11311']);
    rosinit(ipaddress_core,11311, 'NodeHost', ipaddress_local)
    pause(.5)
    laserSub = rossubscriber('/scan');
    odomSub = rossubscriber('/odom');
    cmdPub = rospublisher('/auto_cmd_vel', 'geometry_msgs/Twist');
    pause(.5) % Esperar a que se registren los canales
    cmdMsg = rosmessage(cmdPub);  
end
    

%% Definicion del robot (disco de diametro = 0.35m)
R = 0.072/2;                % Radio de las ruedas [m]
L = 0.235;                  % Distancia entre ruedas [m]
dd = DifferentialDrive(R,L); % creacion del Simulador de robot diferencial

%% Creacion del entorno
load mapa_TP_2025b.mat      %carga el mapa como occupancyMap en la variable 'map'
% load mapa_fiuba_1p.mat      %carga el mapa como occupancyMap en la variable 'map'
% load mapa_lae.mat         %mapa viejo para probar cosas

if verMatlab.Release=='(R2016b)'
    %Para versiones anteriores de MATLAB, puede ser necesario ajustar mapa
    imagen_mapa = 1-double(imread('mapa_fiuba_1p.tiff'))/255;
    map = robotics.OccupancyGrid(imagen_mapa, 25);
elseif verMatlab.Release(1:5)=='(R201'    % Completar con la version que tengan
    %Ni idea que pasa, ver si el truco R2016b funciona
    disp('ver si la compatibilidad R2016b funciona');
else
    disp(['Utilizando MATLAB ', verMatlab.Release]);
end

%% Crear sensor lidar en simulador
lidar = LidarSensor;
lidar.sensorOffset = [0.07,0];     % Posicion del sensor en el robot (asumiendo mundo 2D)
scaleFactor = 3;                %decimar lecturas de lidar acelera el algoritmo
num_scans = ceil(513/scaleFactor);
hokuyo_step_a = deg2rad(-90);
hokuyo_step_c = deg2rad(90);

lidar.scanAngles = linspace(hokuyo_step_a,hokuyo_step_c,num_scans);
lidar.maxRange = 5;

%% Crear visualizacion
viz = Visualizer2D;
viz.mapName = 'map';
attachLidarSensor(viz,lidar);

%% Parametros de la Simulacion

simulationDuration = 60*4;         % Duracion total [s]
sampleTime = 0.1;                   % Sample time [s]
initPose = [18; 15; pi/2];           % Pose inicial (x y theta) del robot simulado (el robot puede arrancar en cualquier lugar valido del mapa)
                                    %  probar iniciar el robot en distintos lugares                                  
                                  
% Inicializar vectores de tiempo:1010
tVec = 0:sampleTime:simulationDuration;         % Vector de Tiempo para duracion total

%% generar comandos a modo de ejemplo
% inicializaciones valores de distancia maxima
dist_frente = 5;
dist_der = 5;
dist_izq = 5;

vxRef = 0.1*ones(size(tVec));   % Velocidad lineal a ser comandada
wRef = zeros(size(tVec));       % Velocidad angular a ser comandada
wRef(tVec < 5) = -0.1;
wRef(tVec >=7.5) = 0.1;

pose = zeros(3,numel(tVec));    % Inicializar matriz de pose
pose(:,1) = initPose;

%% Simulacion

if verMatlab.Release=='(R2016b)'
    r = robotics.Rate(1/sampleTime);    %matlab viejo no tiene funcion rateControl
else
    r = rateControl(1/sampleTime);  %definicion para R2020a, y posiblemente cualquier version nueva
end

for idx = 2:numel(tVec)   
    % Generar aqui criteriosamente velocidades lineales v_cmd y angulares w_cmd
    % -0.5 <= v_cmd <= 0.5 and -4.25 <= w_cmd <= 4.25
    % (mantener las velocidades bajas (v_cmd < 0.1) (w_cmd < 0.5) minimiza vibraciones y
    % mejora las mediciones.   
    
    v_lin_max = 0.1;
    w_max = 0.45;
    dist_segura = 0.8;
    % la distancia critica es en base al tamaño del robot de 0.35m
    dist_critica = 0.4;

    
    % para esta parte se dicidio dividir la medicion del sensor en 3
    % partes, la parte frontral indica la distancia que va a ser
    % restriccion para la velocidad lineal, mientras que las partes de
    % izquierda y derecha de las mediciones del sensor funcionaran para
    % regular las velocidades angulares, evitando asi girar a zonas donde
    % existan obstaculos.  
    
    %% COMPLETAR ACA:
    % generar velocidades para este timestep
        
    % para la velocidad lineal
    if dist_frente < dist_segura
        if dist_frente < dist_critica
            % frena al robot por completo si se acerca mucho a un obstaculo
            v_cmd = 0;
        else
            % regulamos la velocidad en funcion de la distancia segura.
            v_cmd = v_lin_max * (dist_frente/ dist_segura);
            % valor minimo de velocidad para no frenar
            v_cmd = max(0.01,v_cmd);
        end
    else
        v_cmd = v_lin_max;
    end
    
    % caso para velocidad angular
    if dist_frente < dist_segura
        % gira para el lado donde exista la mayor distancia con el
        % obstaculo
        if dist_izq > dist_der
            w_cmd = w_max;
        else
            w_cmd = -w_max;
        end
    else
        % si no hay obstaculo en la parte frontal, no girar.
        w_cmd = 0;
    end
    
    % tope de velocidades maximas
    v_cmd = max(-0.5, min(0.5, v_cmd));
    w_cmd = max(-4.25, min(4.25, w_cmd));
    
    % fin del COMPLETAR ACA
    
    %% a partir de aca el robot real o el simulador ejecutan v_cmd y w_cmd:
    
    if use_roomba       % para usar con el robot real
        
        % Enviar comando de velocidad en el formato que pide el robot:
        cmdMsg.Linear.X = v_cmd;
        cmdMsg.Angular.Z = w_cmd;
        send(cmdPub,cmdMsg);
        
        % Recibir datos de lidar y odometría
        scanMsg = receive(laserSub);
        odompose = odomSub.LatestMessage;
        
        % Obtener vector de distancias del lidar
        ranges_full = laserSub.LatestMessage.Ranges;
        ranges = double(ranges_full(1:scaleFactor:end));
        ranges(ranges==0)=NaN; % lecturas erroneas y maxrange
        ranges(ranges<0.20)=NaN; % compensación por errores de medicion no identificados a Dic/24
        
        % Obtener pose del robot [x,y,yaw] de datos de odometría (integrado por encoders).
        odomQuat = [odompose.Pose.Pose.Orientation.W, odompose.Pose.Pose.Orientation.X, ...
        odompose.Pose.Pose.Orientation.Y, odompose.Pose.Pose.Orientation.Z];
        odomRotation = quat2eul(odomQuat);
        pose(:,idx) = [odompose.Pose.Pose.Position.X + initPose(1); odompose.Pose.Pose.Position.Y+ initPose(2); odomRotation(1)];
    
    else        % para usar el simulador
   
        % Mover el robot segun los comandos generados
        [wL,wR] = inverseKinematics(dd,v_cmd,w_cmd);
        % Velocidad resultante
        [v,w] = forwardKinematics(dd,wL,wR);
        velB = [v;0;w]; % velocidades en la terna del robot [vx;vy;w]
        vel = bodyToWorld(velB,pose(:,idx-1));  % Conversion de la terna del robot a la global
        % Realizar un paso de integracion
        pose(:,idx) = pose(:,idx-1) + vel*sampleTime; 
        % Tomar nueva medicion del lidar
        ranges = double(lidar(pose(:,idx)));
        if simular_ruido_lidar
            % Simular ruido de un lidar ruidoso (probar a ver si se la banca)
            chance_de_medicion_no_valida = 0.17;
            not_valid=rand(length(ranges),1);
            ranges(not_valid<=chance_de_medicion_no_valida)=NaN;
        end
    end
    %%
    % Aca el robot ya ejecutó las velocidades comandadas y devuelve en la
    % variable ranges la medicion del lidar para ser usada y
    % en la variable pose(:,idx) la odometría actual.
    
    %% COMPLETAR ACA:
        % hacer algo con la medicion del lidar (ranges) y con el estado
        % actual de la odometria ( pose(:,idx) ) que se utilizará en la
        % proxima iteración para la generacion de comandos de velocidad
        % ...
        
        % toda lectura menor a 20cm se descarta
        ranges(ranges < 0.20) = NaN; 
        
        % separacion del vector de lecturas en 3 sectores
        num_lecturas = length(ranges);
        tercio = floor(num_lecturas / 3);
        
        % obtencion del obstaculo mas cercano por cada sector ignorando los NaNs
        dist_der_raw = min(ranges(1 : tercio), [], 'omitnan');
        dist_frente_raw = min(ranges(tercio+1 : 2*tercio), [], 'omitnan');
        dist_izq_raw = min(ranges(2*tercio+1 : end), [], 'omitnan');
        
        % correccion de NaNs: Si un sector está libre, se le asigna el rango maximo (5m) 
        if isnan(dist_der_raw),    dist_der = 5;    else, dist_der = dist_der_raw;    end
        if isnan(dist_frente_raw), dist_frente = 5; else, dist_frente = dist_frente_raw; end
        if isnan(dist_izq_raw),    dist_izq = 5;    else, dist_izq = dist_izq_raw; end

        % Fin del COMPLETAR ACA

    % actualizar visualizacion
    viz(pose(:,idx),ranges)
    waitfor(r);
end

