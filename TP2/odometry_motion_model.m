function next_pose = odometry_motion_model (x_t, u_t, alpha)

    %Se separan los datos de la pose del robot
    x = x_t(1);
    y = x_t(2);
    theta = x_t(3);
    
    %Se separan los parámetros de odometría
    delta_r1 = u_t(1);
    delta_r2 = u_t(2);
    delta_t = u_t(3);
    
    %Se separan los parámetros de ruido del modelo
    a1 = alpha(1);
    a2 = alpha(2);
    a3 = alpha(3);
    a4 = alpha(4);
    
    %Se calculan los modulos de los delta para evitar repetir operaciones
    delta_r1_abs = abs(delta_r1);
    delta_r2_abs = abs(delta_r2);
    
    %Se obtiene la varianza para el ruido de cada parámetro de la odometría
    var_r1 = (a1 * delta_r1_abs + a2 * delta_t)^2;
    var_t = (a3 * delta_t + a4 * ( delta_r1_abs + delta_r2_abs))^2;
    var_r2 =(a1 * delta_r2_abs + a2 * delta_t)^2;
    
    %Se calculan los parametros de odometria, con un ruido gaussiano
    delta_hat_r1 = delta_r1 + normal_distribution.sampling_box_muller(0, var_r1, 1);
    delta_hat_t = delta_t + normal_distribution.sampling_box_muller(0, var_t, 1);
    delta_hat_r2 = delta_r2 + normal_distribution.sampling_box_muller(0, var_r2, 1);
    
    %Se obtiene la nueva pose
    x_new = x + delta_hat_t * cos( theta + delta_hat_r1);
    y_new = y + delta_hat_t * sin( theta + delta_hat_r1);
    theta_new = theta + delta_hat_r1 + delta_hat_r2;
    
    next_pose = [x_new; y_new; theta_new];
end