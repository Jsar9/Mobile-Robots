%% PARTE 2

%Pose del robot respecto a la terna global
xr= [ 5,-7,- pi /4];

%Posicion del scaner respecto a la terna del robot
xs = [ 0.2, 0.0, pi];

%FOV del scaner
alpha1 = -pi/2;
alpha2 = - alpha1;
fov=pi;



%Matriz de roto-traslación desde la terna del robot a la terna global
T1 = [ cos(xr(3)) -sin(xr(3)) xr(1); 
       sin(xr(3)) cos(xr(3)) xr(2);
        0 0 1];

%Transformación de la terna global a la terna local del robot
T1_inv = inv(T1);

%Matriz de roto-traslación desde la terna del sensor a la terna del robot.
T12 = [cos(xs(3)) -sin(xs(3)) xs(1); sin(xs(3)) cos(xs(3)) xs(2); 0 0 1 ];

%Matriz de roto-traslación desde la terna del robot a la terna del sensor.
T21= inv(T12);

%% 1
%Muestras del LIDAR respecto del propio sensor

% Se cargan los datos del sensor
data_scan = load('-ascii', 'laserscan.dat');

% Se genera el barrido de los ángulos considerando un step constante
angle = linspace(alpha1, alpha2, size(data_scan,2));

% Se obtienen las coordenadas cartesiadas a partir de los datos del sensor
x_data = data_scan .* cos(angle);
y_data = data_scan .* sin(angle);

% Para graficar el vector de la terna en el origen
origin_vec = 0;
u1=0;
v1=1;
u2=1;
v2=0;

figure();
hold on

%terna del LIDAR
quiver(origin_vec, origin_vec, u1, v1, 0, 'r', 'LineWidth', 2, 'MaxHeadSize', 0.5);
quiver(origin_vec, origin_vec, u2, v2, 0, 'r', 'LineWidth', 4, 'MaxHeadSize', 0.5);
p_lidar = plot(origin_vec, origin_vec,  'ko', 'MarkerSize', 8, 'MarkerFaceColor', 'r');

%datos del escaner
p_scan = plot(x_data, y_data, '.',color = 'blue');
axis equal;
title('Escáner 2D del sensor LIDAR respecto a su propia terna.')
xlabel('$x_{LIDAR}$ [m]', 'Interpreter', 'latex');
ylabel('$y_{LIDAR}$ [m]', 'Interpreter', 'latex');
grid on
xlim([-2,12]);
ylim([-4,10]);
legend([p_lidar, p_scan], {'Terna del LIDAR', 'Muestras'}, 'Location', 'northeast');
exportgraphics(gcf, 'mapeo_lidar_2D.png', 'Resolution', 300);

%% 2
%Posición del robot en la terna global

u=5 *cos(xr(3));
v=5 *sin(xr(3));

% Para graficar el vector de la terna global en el origen
origin_vec = 0;
u1=0;
v1=5;
u2=5;
v2=0;

figure();
hold on

%terna global
quiver(origin_vec, origin_vec, u1, v1, 0, 'r', 'LineWidth', 2, 'MaxHeadSize', 0.5);
quiver(origin_vec, origin_vec, u2, v2, 0, 'r', 'LineWidth', 4, 'MaxHeadSize', 0.5);
p_global = plot(origin_vec, origin_vec, 'ko', 'MarkerSize', 8, 'MarkerFaceColor', 'r');

%pose del robot
quiver(xr(1), xr(2), u, v, 0, 'b', 'LineWidth', 4, 'MaxHeadSize', 0.5);
quiver(xr(1), xr(2), -v, u, 0, 'b', 'LineWidth', 2, 'MaxHeadSize', 0.5);
p_robot = plot(xr(1), xr(2), 'ko', 'MarkerSize', 8, 'MarkerFaceColor', 'b');

axis equal;
title('Posición y orientación del robot respecto de la terna global.')
xlabel('$x_{g}$ [m]', 'Interpreter', 'latex');
ylabel('$y_{g}$ [m]', 'Interpreter', 'latex');
grid on
xlim([-15,15]);
ylim([-15,15]);
legend([p_global, p_robot], {'Terna Global', 'Posición del Robot'}, 'Location', 'northeast');
exportgraphics(gcf, 'pose_robot_global.png', 'Resolution', 300);

%% 3
%Posición del LIDAR respecto de la terna global
u=5 *cos(xs(3));
v=5 *sin(xs(3));

xs_g = T1 * [xs(1);xs(2);1];

origin_vec = 0;
u1=0;
v1=5;
u2=5;
v2=0;

figure();
hold on
% terna global
quiver(origin_vec, origin_vec, u1, v1, 0, 'r', 'LineWidth', 2, 'MaxHeadSize', 0.5);
quiver(origin_vec, origin_vec, u2, v2, 0, 'r', 'LineWidth', 4, 'MaxHeadSize', 0.5);
p_global = plot(origin_vec, origin_vec, 'ko', 'MarkerSize', 8, 'MarkerFaceColor', 'r');

% pose del LIDAR
quiver(xs_g(1), xs_g(2), u, v, 0, 'LineWidth', 4, 'MaxHeadSize', 0.5, 'Color', 'b');
quiver(xs_g(1), xs_g(2), -v, u, 0, 'LineWidth', 2, 'MaxHeadSize', 0.5, 'Color', 'b');
p_lidar = plot(xs_g(1), xs_g(2), 'ko', 'MarkerSize', 8, 'MarkerFaceColor', 'b');


axis equal;
title('Posición y orientación del LIDAR respecto de la terna global.')
xlabel('$x_{g}$ [m]', 'Interpreter', 'latex');
ylabel('$y_{g}$ [m]', 'Interpreter', 'latex');
grid on
xlim([-15,15]);
ylim([-15,15]);
legend([p_global, p_lidar], {'Terna Global', 'Posición del LIDAR'}, 'Location', 'northeast');
exportgraphics(gcf, 'pose_lidar_global.png', 'Resolution', 300);

%% 4
%Posición de las mediciones del LIDAR respecto de la terna global}
p_scan = [x_data; y_data; ones(1,size(x_data,2))];

% Se realiza la transformación de la terna del LIDAR a la del robot y del
% robot a la global
xs_g = T1 * T12 * p_scan;

origin_vec = 0;
u1=0;
v1=1;

u2=1;
v2=0;

figure();
hold on
%terna global
quiver(origin_vec, origin_vec, u1, v1, 0, 'r', 'LineWidth', 2, 'MaxHeadSize', 0.5);
quiver(origin_vec, origin_vec, u2, v2, 0, 'r', 'LineWidth', 4, 'MaxHeadSize', 0.5);
p_global = plot(origin_vec, origin_vec, 'ko', 'MarkerSize', 8, 'MarkerFaceColor', 'r');

%muestras
p_scan = plot(xs_g(1,:), xs_g(2,:), '.', color='b');
axis equal;
title('Escáner 2D del sensor LIDAR respecto a la terna global.')
xlabel('$x_{g}$ [m]', 'Interpreter', 'latex');
ylabel('$y_{g}$ [m]', 'Interpreter', 'latex');
grid on
legend([p_global, p_scan], {'Terna Global', 'Muestras'}, 'Location', 'northeast');
exportgraphics(gcf, 'mediciones_lidar_global.png', 'Resolution', 300);

%% PARTE 3

% Posicion y parametros iniciales
x = 0.0;
y = 0.0;
theta = pi/4; 
l = 0.5;      
dt = 0.05; % Intervalos de tiempo para graficar la trayectoria

% Se definen las acciones que sigue el robot
actions = [
    0.1,  0.5, 2;  % c1
    0.5,  0.1, 2;  % c2
    0.2,  0.2, 2;  % c3
    1.0,  0.0, 4;  % c4
    0.4,  0.4, 2;  % c5
    0.2, -0.2, 2;  % c6
    0.5,  0.5, 2   % c7
];

% Se crea un array inicial para mejorar la eficiencia de memoria del código
% con respecto a la implementación previa (Matlab daba warning por hacer realloc
% de memoria en cada iteración dt del bucle temporal)
max_size = 10000; %tomo un valor grande para evitar problemas
x_hist = zeros(1, max_size);
y_hist = zeros(1, max_size);


% Se inicializa el historial para las trayectorias usando la posicion
% inicial
x_hist(1) = x;
y_hist(1) = y;
counter = 1; % Se utiliza un contador para recortar los ceros sobrantes al final

%Se obtiene la cantidad de acciones a realizar
n_actions = size(actions, 1);

% Ciclo que corre todas las acciones
for i = 1:n_actions
    
    % Se cargan las velocidades y tiempos de la accion actual
    v_l = actions(i, 1);
    v_r = actions(i, 2);
    t_total = actions(i, 3);
    
    % Ciclo que recorre el tiempo total de cada acción en intervalos dt
    for t_step = 0:dt:t_total
        
        % Se obtiene la pose en la propia terna del robot
        [x_local, y_local, theta_local] = diffdrive(x, y, theta, v_l, v_r, dt, l);

        % Se crea el vector de la posicion local, usado para la
        % transformación
        P_local = [x_local; y_local; 1];
        
        % Matriz de roto-traslación desde la terna del robot a la terna global
        % se actualiza en cada dt, tomando la pose actual global del robot
        T1 = [ cos(theta) -sin(theta) x; 
                sin(theta)  cos(theta) y; 
                0 0 1];
        
        % Se obtiene la pose en la terna global del robot mediante la 
        % transformacion actualizada a la última pose, T1
        P_global = T1 * P_local;
        
        % Se obtienen las coordenadas globales del robot
        x = P_global(1);
        y = P_global(2);
        
        % Se actualiza el ángulo del robot según la variación en cada dt
        theta = theta + theta_local;
        
        % Se guardan las coordenadas del robot con respecto a la terna
        % global en el historial
        counter = counter + 1; 
        x_hist(counter) = x;
        y_hist(counter) = y;
    end
end

% Se recortan los ceros sobrantes
x_hist = x_hist(1:counter);
y_hist = y_hist(1:counter);

origin_x = 0.0;
origin_y = 0.0;
origin_u = 0.1*cos(pi/4);
origin_v = 0.1*sin(pi/4);

final_x = x; %toma el ultimo valor del historial de la trayectoria
final_y = y;
u_final=0.1 * cos(theta);
v_final=0.1 * sin(theta);

figure;
hold on

%pose inicial
quiver(origin_x, origin_y, origin_u, origin_v, 0, 'r', 'LineWidth', 4, 'MaxHeadSize', 0.5);
quiver(origin_x, origin_y, -origin_v, origin_u, 0, 'r', 'LineWidth', 2, 'MaxHeadSize', 0.5);
p_init = plot(origin_x,origin_y,'ko', 'MarkerSize', 8, 'MarkerFaceColor', 'r');

%pose final
quiver(final_x, final_y, u_final, v_final, 0, 'g', 'LineWidth', 4, 'MaxHeadSize', 0.5);
quiver(final_x, final_y, -v_final, u_final, 0, 'g', 'LineWidth', 2, 'MaxHeadSize', 0.5);
p_final = plot(final_x,final_y,'ko', 'MarkerSize', 8, 'MarkerFaceColor', 'g');

%trayectoria
p_w = plot(x_hist, y_hist, 'b-', 'LineWidth', 1, color = 'b');
hold on;
grid on;
xlabel('$x_{g}$ [m]', 'Interpreter', 'latex');
ylabel('$y_{g}$ [m]', 'Interpreter', 'latex');
title('Trayectoria respecto a la terna global');
legend([p_init, p_final, p_w], {'Pose Inicial', 'Pose Final', 'Trayectoria' }, 'Location', 'northeast');
axis equal;
exportgraphics(gcf, 'trayectoria.png', 'Resolution', 300);
