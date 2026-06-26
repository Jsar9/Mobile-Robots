function n = neighbors(cell, map_dimensions)

  n = [];

  pos_x = cell(2);
  pos_y = cell(1);
  size_x = map_dimensions(2);
  size_y = map_dimensions(1);
  
  %%% YOUR CODE FOR CALCULATING THE NEIGHBORS OF A CELL GOES HERE
  
  % Return nx2 vector with the cell coordinates of the neighbors. 
  % Because planning_framework.m defines the cell positions as pos = [cell_y, cell_x],
  % make sure to return the neighbors as [n1_y, n1_x; n2_y, n2_x; ... ]
  
  %Se crea una matriz con los movimientos hacia cada una de las 8
  %direcciones de la celda. Cuantas debe sumar o restar al número de fila o columna
  %desde su posición para llegar a los nodos adyacentes.
  movements = [
      -1,  0; % Arriba
       1,  0; % Abajo
       0, -1; % Izquierda
       0,  1; % Derecha
      -1, -1; % Arriba-Izquierda
      -1,  1; % Arriba-Derecha
       1, -1; % Abajo-Izquierda
       1,  1  % Abajo-Derecha
  ];


  %Se recorre cada una de las celdas adyacentes a la celda dada y se guardan las
  %coordenadas de cada una en la matriz n.
  for i = 1:size(movements, 1)
      
      %Se obtienen las coordenadas del nodo adyacente.
      n_y = pos_y + movements(i, 1);
      n_x = pos_x + movements(i, 2);
        
      %Se comprueba que el nodo se encuentre dentro de los límites del
      %mapa.
      if (n_y >= 1 && n_y <= size_y) && (n_x >= 1 && n_x <= size_x)
          
          %Si está en el límite del mapa, se guardan las coordenadas en la
          %matriz n (de nx2) junto a las coordenadas previas.
          n = [n; n_y, n_x];
      end
   end
end
