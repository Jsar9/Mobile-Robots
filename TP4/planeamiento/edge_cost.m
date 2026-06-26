function cost = edge_cost(parent, child, map)

  cost = 0;
 
  %%% YOUR CODE FOR CALCULATING THE COST FROM VERTEX parent TO VERTEX child GOES HERE
  
  OCCUPANCY_THRESHOLD = 0.2;

  
  %Se separan las coordenadas del nodo padre (nodo actual)
  parent_x = parent(2);
  parent_y = parent(1);
  
  %Se separan las coordenadas del nodo hijo (siguiente nodo)
  child_x = child(2);
  child_y = child(1);
  
  %Se calcula la distancia euclidiana del padre al hijo.
  % Si el hijo está en la posición horizontal o vertical, dist = 1
  % Si el hijo está en alguna de las diagonales, valdra dist = sqrt(2) =
  % 1.41
  dist = sqrt( (child_x - parent_x)^2 + (child_y - parent_y)^2 );
  
  %Se obtiene el valor de esa celda hijo en el mapa (su probabilidad de ocupación)
  map_value = map(child_y, child_x);
  
  %Si la probabilidad de ocupación de la celda supera el umbral, entonces
  %el costo es infinito, lo que impide que el robot la considere como ruta.
  if map_value > OCCUPANCY_THRESHOLD
      cost = inf;
      return;
  end
  
  %Se determina el factor de penalización, que aumentará el costo según la
  %probabilidad de que la celda se encuentre ocupada
  penalty_f = 10;
  
  %Se determina el costo hacia la celda en base a la distancia, la
  %probabilidad de ocupación y el factor de penalidad
  cost = dist + (map_value * penalty_f);
  
end
