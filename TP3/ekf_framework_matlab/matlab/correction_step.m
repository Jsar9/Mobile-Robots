function [mu, sigma] = correction_step(mu, sigma, z, l)
    % Updates the belief, i. e., mu and sigma, according to the sensor model
    %
    % The employed sensor model is range-only.
    %
    % mu: 3 x 1 vector representing the mean (x, y, theta) of the normal distribution
    % sigma: 3 x 3 covariance matrix of the normal distribution
    % z: structure containing the landmark observations, see
    %    read_data for the format
    % l: structure containing the landmark position and ids, see
    %    read_world for the format

    %Se extraen las coordenadas del robot para mayor legibilidad del código
    x = mu(1);
    y = mu(2);
    

    % Compute the expected range measurements.
    % This corresponds to the function h.
    expected_ranges = zeros(size(z, 2), 1);
    for i = 1:size(z, 2)
        % Todo: Implement
        %Se obtienen las coordenadas del landmark actual
        lx = l(z(i).id).x;
        ly = l(z(i).id).y;
        
        %Se calcula la distancia esperada
        expected_ranges(i) = sqrt((lx - x)^2 + (ly - y)^2);
        
    end

    % Jacobian of h
    H = zeros(size(z, 2), 3);

    % Measurements in vectorized form
    Z = zeros(size(z, 2), 1);
    for i = 1:size(z, 2)
        
        % Todo: Implement
        
        lx = l(z(i).id).x;
        ly = l(z(i).id).y;
        
        
        %Matriz Jacobiana H, compuesta por las derivadas de las distancias
        %esperadas
        H(i, :) = [(x - lx) / expected_ranges(i), (y - ly) / expected_ranges(i), 0];
        
        %Se almacena en Z las lecturas reales del sensor
        Z(i) = z(i).range; 
    end

    R = diag(repmat([0.5], size(z, 2), 1));
    
    % Todo: Implement
    %Ganancia del filtro de Kalman
    K = sigma * H' * inv(H * sigma * H' + R);
    
    %Se suma a la pose estimada, el error de medición escalado por K
    mu = mu + K * (Z - expected_ranges);
    
    %Se actualiza la matriz de covarianzas
    sigma = (eye(3) - K * H) * sigma;
end
