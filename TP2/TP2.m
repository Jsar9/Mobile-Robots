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

n = 5000;
x_t = [2.0; 4.0; pi/2];
u_t = [pi/4; 0.0; 1.0];
alpha = [0.1; 0.1; 0.01; 0.01];

x_samples = zeros(3, n);

for i = 1:n
    x_samples(:, i) = odometry_motion_model(x_t, u_t, alpha);
end

figure();
hold on;
grid on;

x_new = x_samples(1, :);
y_new = x_samples(2, :);

scatter(x_new, y_new, 5, 'b', 'filled', 'MarkerFaceAlpha', 0.5);

plot(x_t(1), x_t(2), 'MarkerSize', 20, 'MarkerFaceColor', 'r');

title('Modelo de odometría');
xlabel('X');
ylabel('Y');
legend('Muestras', 'Pose inicial', 'Location', 'best');
axis equal;
hold off;


%% PARTE 3

% Filtro discreto




