function mean_pos = mean_position(particles, weights)
    % Returns a single estimate of filter state based on the particle cloud.
    %
    % particles (M x 3): set of M particles to sample from. Each row contains a state hypothesis of dimension 3 (x, y, theta).
    % weights (M x 1): weights of the particles. Each row contains a weight.

    % initialize
    mean_pos = zeros(1,3);

    %% TODO: compute mean_pos    
    
    %Se calcula la media para las coordenadas x e y
    mean_pos(1) = weights' * particles(:, 1);
    mean_pos(2) = weights' * particles(:, 2);
    
    %Se obtienen los angulos
    angle = particles(:, 3);
    
    %Se aplica el peso de cada partícula por la trigonométrica de cada
    %ángulo y suma los resultados
    vx = weights' * cos(angle);
    vy = weights' * sin(angle);
    
    %Se obtiene la media de los ángulos, mediante atan2 evitando problemas
    %entre los cuadrantes
    mean_pos(3) = atan2(vy, vx);
end
