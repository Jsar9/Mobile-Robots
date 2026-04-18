%Función que implementa la cinemática directa de un robot con accionamiento diferencial
%RETORNA LOS VALORES CORRESPONDIENTES A LA TERNA DEL ROBOT
function [x_n y_n theta_n] = diffdrive(x, y, theta, v_l, v_r, t, l)
    x_dot = 0.5 * v_l + 0.5 * v_r;
    
    theta_dot = v_r/l - v_l/l;
     
    x_n = x_dot * t;
    theta_n = theta_dot * t;
    y_n = 0;
end





