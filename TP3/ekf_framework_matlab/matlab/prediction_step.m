function [mu, sigma] = prediction_step(mu, sigma, u)
    % Updates the belief, i. e., mu and sigma, according to the motion model
    %
    % u: odometry reading (r1, t, r2)
    % mu: 3 x 1 vector representing the mean (x, y, theta) of the normal distribution
    % sigma: 3 x 3 covariance matrix of the normal distribution
    
    %Se extraen los datos para mayor legibilidad del código
    x = mu(1);
    y = mu(2);
    theta = mu(3);
    
    r1 = u.r1;      % delta_rot1
    trans = u.t;    % delta_trans
    r2 = u.r2;      % delta_rot2
    
    % Compute the noise-free motion. This corresponds to the function g, evaluated
    % at the state mu.
    %impl
    mu = [ x + trans * cos(theta + r1);
           y + trans * sin(theta + r1);
           theta + r1 + r2 ];

    % Compute the Jacobian of g with respect to the state
    %impl
    G = [ 1, 0, -trans * sin(theta + r1);
          0, 1,  trans * cos(theta + r1);
          0, 0,  1 ];

    % Motion noise
    Q = [0.2, 0, 0; 
        0, 0.2, 0; 
        0, 0, 0.02];
    %Se actualiza la matriz de covarianza
    %impl
    sigma = G * sigma * G' + Q;
end
