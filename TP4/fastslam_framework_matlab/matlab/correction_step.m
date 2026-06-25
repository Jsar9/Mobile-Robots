function particles = correction_step(particles, z)

% Weight the particles according to the current map of the particle
% and the landmark observations z.
% z: struct array containing the landmark observations.
% Each observation z(j) has an id z(j).id, a range z(j).range, and a bearing z(j).bearing
% The vector observedLandmarks indicates which landmarks have been observed
% at some point by the robot.

% Number of particles
numParticles = length(particles);

% Number of measurements in this time step
m = size(z, 2);

% TODO: Construct the sensor noise matrix Q_t (2 x 2)
% Ruido del sensor
Q_t = [0.1 0; 0 0.1];


% process each particle
for i = 1:numParticles
  robot = particles(i).pose;
  % process each measurement
  for j = 1:m
    % Get the id of the landmark corresponding to the j-th observation
    % particles(i).landmarks(l) is the EKF for this landmark
    l = z(j).id;

    % The (2x2) EKF of the landmark is given by
    % its mean particles(i).landmarks(l).mu
    % and by its covariance particles(i).landmarks(l).sigma

    % If the landmark is observed for the first time:
    if (particles(i).landmarks(l).observed == false)

      % TODO: Initialize its position based on the measurement and the current robot pose:
      r = z(j).range;
      phi = z(j).bearing;
      x_rob = robot(1);
      y_rob = robot(2);
      theta_rob = robot(3);
      
      %Se mapea la pose del landmark a la terna global según la pose
      %actual del robot
      particles(i).landmarks(l).mu = [x_rob + r * cos(theta_rob + phi); 
                                      y_rob + r * sin(theta_rob + phi)];

      % get the Jacobian with respect to the landmark position
      [h, H] = measurement_model(particles(i), z(j));

      % TODO: initialize the covariance for this landmark
      %Se calcula la varianza para el landmark l, utilizando la matriz
      %inversa de la jacobiana H y su transpuesta
      particles(i).landmarks(l).sigma = inv(H) * Q_t * inv(H)';

      % Indicate that this landmark has been observed
      particles(i).landmarks(l).observed = true;

    else

      % get the expected measurement
      [expectedZ, H] = measurement_model(particles(i), z(j));

      % TODO: compute the measurement covariance
      %Se obtiene la incertidumbre total, producto del sensor y la
      %incertidumbre asociada a la posición del landmark
      Q = H * particles(i).landmarks(l).sigma * H' + Q_t;

      % TODO: calculate the Kalman gain
      %Se calcula la ganancia de Kalman
      K = particles(i).landmarks(l).sigma * H' * inv(Q);

      % TODO: compute the error between the z and expectedZ (remember to normalize the angle using the function normalize_angle())
      % Se obtiene el error de la medición del sensor con lo que se esperaba medir
      % basándose en su mapa (Tanto la distancia como el ángulo medido y
      % esperado).
      diffZ = [z(j).range - expectedZ(1);
               normalize_angle(z(j).bearing - expectedZ(2))];

      % TODO: update the mean and covariance of the EKF for this landmark
      %Se actualizan las medias y covarianzas del EKF, utilizando la
      %ganancia de Kalman y la diferencia previa.
      particles(i).landmarks(l).mu = particles(i).landmarks(l).mu + K * diffZ;
      particles(i).landmarks(l).sigma = (eye(2) - K * H) * particles(i).landmarks(l).sigma;

      % TODO: compute the likelihood of this observation, multiply with the former weight
      %       to account for observing several features in one time step
      %Se calcula la verosimilitud, utilizando la expresión de la Gaussiana
      likelihood = (1 / sqrt(det(2 * pi * Q))) * exp(-0.5 * diffZ' * inv(Q) * diffZ);
      particles(i).weight = particles(i).weight * likelihood;

    end

  end % measurement loop
end % particle loop

end
