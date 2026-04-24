%% PARTE 1
% MUESTREO DE DISTRIBUCIONES DE PROBABILIDAD

N = 1e6;
mu = 2;
var = 4;
sigma = sqrt(var);

%Se comprueba el tiempo del método de suma
tic;
normal_distribution.sampling_sum(mu, var, N);
time_sum = toc;
fprintf('1 - Tiempo del método suma:    %.4f segundos\n', time_sum);

%Se comprueba el tiempo del método de rechazo
tic;
normal_distribution.sampling_rejection(mu, var, N);
time_rej = toc;
fprintf('2 - Tiempo del método de rechazo:    %.4f segundos\n', time_rej);

%Se comprueba el tiempo de lmétodo de Box-Muller
tic;
normal_distribution.sampling_box_muller(mu, var, N);
time_bm = toc;
fprintf('3 - Tiempo del método de Box-Muller:    %.4f segundos\n', time_bm);

%Se comprueba el tiempo del método normrnd
tic;
normrnd(mu,sigma,1,N);
time_nr = toc;
fprintf('4 - Tiempo del método normrnd:    %.4f segundos\n', time_nr);



%% PARTE 2
% MODELO BASADO EN ODOMETRÍA

%Inicializacion de parametros
n = 5000;
x_t = [2.0; 4.0; pi/2];
u_t = [pi/4; 0.0; 1.0];
alpha = [0.1; 0.1; 0.01; 0.01];

x_samples = zeros(3, n);

%Se toman las muestras del movimiento
for i = 1:n
    x_samples(:, i) = odometry_motion_model(x_t, u_t, alpha);
end

%Grafico
figure();
hold on;
grid on;
x_new = x_samples(1, :);
y_new = x_samples(2, :);

amp = 0.3;
x_r = amp * cos(x_t(3)); 
y_r = amp * sin(x_t(3));

scatter(x_new, y_new, 5, 'b', 'filled', 'MarkerFaceAlpha', 0.5);

quiver(x_t(1), x_t(2), x_r, y_r, 0, 'LineWidth', 2)
plot(x_t(1), x_t(2), 'ro', 'MarkerFaceColor', 'r');

title('Modelo basado en odometría');
xlabel('X');
ylabel('Y');
legend('Muestras', 'Pose inicial', 'Location', 'best');
axis equal;
hold off;


%% PARTE 3
% FILTRO DISCRETO

%cantidad de celdas
n = 20;

% Se inicializa el belief como un vector columa
% Se tiene probabilidad 1 para el elemento 11 (correspondiente a la celda 10)
bel = [zeros(10, 1); 1; zeros(9, 1)]; % 20x1

% Se trabajan los avances y retrocesos como matrices de nxn

% La matriz de avances, tiene para cada celda i, la probabilidad
% correspondiente a permanecer en dicha celda y las probabilidades de
% avanzar a las siguientes celdas según corresponda cada caso
M_forward = zeros(n,n);

M_backward = zeros(n,n);


for i = 1:n
    
    % CASOS DE AVANCE
    % Casos previos a la penultima celda
    if i <= n - 2
        M_forward(i, i)   = 0.25; % No avanza
        M_forward(i+1, i) = 0.50; % Avanza 1 celda
        M_forward(i+2, i) = 0.25; % Avanza 2 celdas
        
    % Caso de la penultima celda    
    elseif i == n - 1
        M_forward(i, i)   = 0.25; % No ava<nza
        M_forward(i+1, i) = 0.75; % Avanza a la ultima celda
    
    % Caso de la ultima celda (extremo derecho)
    elseif i == n
        M_forward(i, i)   = 1.00; % No avanza
    end
    
    %CASOS DE RETROCESO
    % Casos desde la tercer celda en adelante
    if i >= 3
        M_backward(i, i)   = 0.25; % No retrocede
        M_backward(i-1, i) = 0.50; % Retrocede 1 celda
        M_backward(i-2, i) = 0.25; % Retrocede 2 celdas
        
    % Caso de la 2da celda
    elseif i == 2
        M_backward(i, i)   = 0.25; % No retrocede
        M_backward(i-1, i) = 0.75; % Retrocede a la primer celda
        
    % Caso de la primer celda (extremo izquierdo)
    elseif i == 1
        M_backward(i, i)   = 1.00; % No retrocede
    end
end
    
% Se ejecutan los comandos de avance, incrementando el belief en cada celda
% según la informacion de la matriz de avance, utilizando la probabilidad
% total ( sum(p * bel) )
for k = 1:9
    bel = M_forward * bel;
end

% Se ejecutan los comandos de retroceso
for k = 1:3
    bel = M_backward * bel;
end



figure();
bar(0:19, bel, 'b');
title('Belief tras los comandos');
xlabel('Celdas');
ylabel('Belief - Probabilidad');
grid on;
axis([-1 20 0 max(bel)+0.05]);
