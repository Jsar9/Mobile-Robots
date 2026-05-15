function weight = measurement_model(z, x, l)
    % Computes the observation likelihood of all particles.
    %
    % The employed sensor model is range only.
    %
    % z: set of landmark observations. Each observation contains the id of the landmark observed in z(i).id and the measured range in z(i).range.
    % x: set of current particles
    % l: map of the environment composed of all landmarks
    sigma = [0.2];
    weight = ones(size(x, 1), 1);

    if size(z, 2) == 0
        return
    end
    
    for i = 1:size(z, 2)
        landmark_position = [l(z(i).id).x, l(z(i).id).y];
        measurement_range = [z(i).range];

        %% TODO: compute weight
        %Se calcula la posición entre cada una de las partículas y el
        %landmark, es la distancia que se espera medir con el sensor 
        %si el robot se encuentra en la posición de la partícula
        exp_distance = sqrt((x(:,1) - landmark_position(1)).^2 + (x(:,2) - landmark_position(2)).^2);
        
        %Se calcula que tan creíble es que el robot sea la partícula
        %según la posición relativa al landmark, usando la PDF de una Gaussiana
        %como ruido para la partícula
        likelihood = (1 / sqrt(2 * pi * sigma^2)) * exp(-((measurement_range - exp_distance).^2) / (2 * sigma^2));
        
        weight = weight .* likelihood;
        
    end

    weight = weight ./ size(z, 2);
end
