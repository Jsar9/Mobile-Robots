%Pose del robot respecto a la terna global
xr= [ 5,-7,- pi /4];

%Posicion del scaner respecto a la terna del robot
xs = [ 0.2, 0.0, pi];

%FOV del scaner
alpha1 = -pi/2;
alpha2 = - alpha1;
fov=pi;

%Matriz de roto-traslación desde la terna del robot a la terna global
T1= [cos(xr(3)) -sin(xr(3)) xr(1); sin(xr(3)) cos(xr(3)) xr(2); 0 0 1 ];

%Transformación de la terna global a la terna local del robot
T1_inv = inv(T1);

%Matriz de roto-traslación desde la terna del sensor a la terna del robot.
T12 = [cos(xs(3)) -sin(xs(3)) xs(1); sin(xs(3)) cos(xs(3)) xs(2); 0 0 1 ];

%Matriz de roto-traslación desde la terna del robot a la terna del sensor.
T21= inv(T12);

data_scan = load('-ascii', 'laserscan.dat');

angle = linspace(-pi/2, pi/2, size(data_scan,2));

x_s = data_scan .* cos(angle);
y_s = data_scan .* sin(angle);

figure();
plot(x_s, y_s, '.');
axis equal;
title('Escáner 2D del sensor LIDAR respecto a su propia terna.')
xlabel('$x_{LIDAR}$ [m]', 'Interpreter', 'latex');
ylabel('$y_{LIDAR}$ [m]', 'Interpreter', 'latex');
exportgraphics(gcf, 'mapeo_lidar_2D.png', 'Resolution', 300);

