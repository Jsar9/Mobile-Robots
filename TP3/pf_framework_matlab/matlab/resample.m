function new_particles = resample(particles, weights)
    % Returns a new set of particles obtained by performing
    % stochastic universal sampling.
    %
    % particles (M x D): set of M particles to sample from. Each row contains a state hypothesis of dimension D.
    % weights (M x 1): weights of the particles. Each row contains a weight.
    
    %Se fija el tamaño de new_particles para evitar reasignar memoria en el
    %bucle for
    M = size(particles, 1);
    D = size(particles, 2);
    
    new_particles = zeros(M, D);
    
    %% TODO: complete this stub
    
    %Se obtiene el step según la cantidad de partículas
    step = 1 / M;
    
    %Se obtiene un número aleatorio para fijar el inicio
    r = rand() * step;
    
    %C es el acumulador de los pesos, se lo inicializa con el valor del
    %peso de la primera partícula
    c = weights(1);
    
    %id de la particula
    i = 1;
    
    %Se recorren las M particulas 
    for m = 1:M
        
        %Se calcula la nueva posición partiendo de r
        U = r + (m - 1) * step;
        
        %Si la nueva posicion supera al peso acumuilado c (cayo por fuera del rango [0,c] )
        while U > c
            i = i + 1; %Se saltea a la siguiente particula
            c = c + weights(i); %Se agranda el limite de los pesos con el acumulador
        end
        
        %Se copia la partícula con mayor peso acumulado (aquellas que U <c)
        new_particles(m, :) = particles(i, :);
    end
end
