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








