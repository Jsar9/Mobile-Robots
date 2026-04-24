classdef normal_distribution
    % Se agrupan las funciones de la primer parte del TP2
    methods (Static)
        
        
        function values = sampling_sum(mu,var, N)
            values = zeros(1,N);
            sigma = sqrt(var);
            
            for i=1:N
                
                % Se calcula la suma de las 12 muestras de la uniforme
                % entre -sigma y sigma
                rand_values = -sigma + (2 * sigma) * rand(1, 12);
                
                
                values(i) = mu + 0.5 * sum(rand_values);
            end
        end
        
        function values = sampling_rejection (mu,var,N)
            values = zeros(1,N);
            sigma = sqrt(var);
            
            %Se obtiene el máximo de la PDF normal
            M = 1 / sqrt(2*pi);
            
            %Se definen los limites para el muestreo de la normal,
            %buscandoabarcar el mayor área posible partiendo de sigma
            b = sigma+5;
            
            %Se definen las muestras que se aceptaron (debe llegar a N)
            samples_accepted = 0;
            
            %Se ejecuta hasta obtener las N muestras
            while samples_accepted < N
                
                %Se generan las muestras x e y usando las uniformes en los
                %intervalos 
                x = -b + (2*b) * rand(); % de -b a b
                y = M * rand(); % de 0 a M
                
                %Se obtiene la PDF en los x obtenidos
                f_x = (1 / sqrt(2*pi)) * exp(-0.5 * x^2);
                
                %Se comparan las muestras y con el valor de la PDF en cada
                %x
                if y <= f_x
                    samples_accepted = samples_accepted + 1;
                    values(samples_accepted) = mu + x * sigma;
                end
            end
        end
        
        function values = sampling_box_muller(mu, var, N)
            
            sigma = sqrt(var);
            
            %Se obtienen las 2 uniformes necesarias
            u1 = rand(1, N);
            u2 = rand(1, N);
            
            %Se obtienen las muestras utilizando u1 y u2
            x = cos(2 * pi * u1) .* sqrt(-2 * log(u2));
            
            values = mu + x * sigma;
        end
        
    end
end