function heur = heuristic(cell, goal)
  
  heur = 0;
  
  %%% YOUR CODE FOR CALCULATING THE REMAINING COST FROM A CELL TO THE GOAL GOES HERE
  
  %¿Qué propiedades debe tener la heurística para asegurar que A* sea óptimo?
  % Debe cumplir la condición de admisibilidad:
  %     -No debe sobreestimar la distancia real hacia el destino.
  %     -Debe ser optimista. Si por ejemplo la distancia en línea recta hacia el destino
  %         es de 10m, la heurística debe dar una distancia menor,como 8m, para evitar
  %         que el algoritmo ignore esas potenciales rutas óptimas por un falso
  %         costo elevado.
  % Además, debe ser consistente:
  %     -El costo estimado desde un nodo hasta el destino (en linea recta),
  %         no debe ser mayor que el costo de ir hacia un nodo vecino sumado
  %         al costo desde el vecino hasta el destino. Es decir, debe cumplir
  %         con una desigualdad triangular con los costos.
  
  
  %Se extraen las coordenadas de la celda origen
  cell_x = cell(2);
  cell_y = cell(1);
  
  %Se extraen las coordenadas de la celda destino
  goal_x = goal(2);
  goal_y = goal(1);
  
  %Se calcula la heurística utilizando la distancia euclidiana hasta la
  %meta. Es decir, en línea recta desde la celda origen hasta la celda
  %destino.
  
  k=10;
  heur = k*sqrt((cell_x - goal_x)^2 + (cell_y - goal_y)^2);
  
  %Resultado con k=1:
  % path cost: 
  %    36.9488
  % 
  % path length: 
  %    36.3848
  % 
  % number of nodes visited: 
  %    354
  
  
  %Resultado con k=2:
  % path cost: 
  %    38.2489
  % 
  % path length: 
  %    35.7990
  % 
  % number of nodes visited: 
  %     73
  
  
  %Resultado con k=5:
  % path cost: 
  %    42.6816
  % 
  % path length: 
  %    38.6274
  % 
  % number of nodes visited: 
  %     45
  
  
  %Resultado con k=10:
  % path cost: 
  %    44.2223
  % 
  % path length: 
  %    39.4558
  % 
  % number of nodes visited: 
  %     40
  
  
  %Con k=1, el algoritmo es muy lento, pero alcanza el objetivo de
  %manera exitosa.
  
  %Con k=2 el algoritmo es muchísimo más rápido que con k=1, alcanzando
  %también el objetivo de manera exitosa y con muchos menos nodos visitados.
  
  %Con k=5 el algoritmo es rápido, pero el cambio en la trayectoria, eleva
  %el costo de la ruta encontrada.
  
  %Con k=10, el algoritmo es rápido, pero nuevamente eleva el costo, la
  %ruta encontrada es menos óptima que las anteriores a pesar de ser muy
  %rápido por visitar pocos nodos.
  
  
  
  %Al aumentar k, disminuye la cantidad de nodos visitados, por lo que el
  %algoritmo encuentra más rápido una ruta hacia la celda destino. Por otro
  %lado, el costo de la ruta óptima aumenta a medida que aumenta el valor
  %de la constante, ya que el algoritmo tiende a sobreajustar la ruta
  %hacia el destino a medida que avanza entre nodos, generando en
  %consecuencia un camino más largo.
  %Se observó que subir el umbral de ocupación o bajar la penalización,
  %puede provocar que con el aumento de k, el algoritmo tienda a atravesar
  %zonas con alta tasa de ocupación o directamente zonas grises, sin
  %información, por lo que aumenta enormemente el costo de la ruta óptima.
  %Jugando con dichos parámetros, se puede hallar una relación adecuada
  %entre la ruta óptima, con un costo bajo y la menor cantidad de nodos
  %visitados posibles.
  
end
