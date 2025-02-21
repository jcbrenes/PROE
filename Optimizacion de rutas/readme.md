En la carpeta Processing se encuentran los documentos mayormente relacionados al procesamiento de datos.

Dentro se encuentra la carpeta mapeo en donde están todos los archivos. Esto es porque processing tiene la similitud con Arduino de que el archivo mapeo.pde debe estar dentro de una carpeta llamada igual, o sea, mapeo.

En este directorio deben estar las carpetas de videos "1_robot", "2_robot" y "3_robot" que corresponden a procesamiento offline. También está el programa de processing, "mapeo.pde", el cual funciona de la siguiente forma:
1. Se ejecuta el script clickeando en la flecha en la parte superior del programa.
2. Se da click al círculo grande de cada robot para comenzar a seguir ese color.
En ese entonces se crea una carpeta llamada "prueba" (o algo así) dentro de la cual se generan las carpetas "divided_color_matrix" y "reduced_color_matrix", dentro de las cuales, asimismo, se generan las matrices correspondientes, cada 10 segundos. Dentro de esta carpeta se debe poner el video grabado de dicha prueba y nombrar la carpeta de forma adecuada al final de la prueba.

Además, está el programa de R, "Graficas", que se encarga de realizar los gráficos de análisis estadístico con base en los archivos "Principal" y "Resumen". Estos dos archivos son generados por un programa de Python de otra carpeta que se explica a continuación.

----------------------------------------------------------

En la carpeta Python se encuentra el programa para realizar el procesamiento de las matrices generadas por Processing. Si se desea realizar el procesamiento en tiempo real, se debe abrir el programa "main ONLINE.py" después de abrir "mapeo.pde". Este programa genera en la carpeta "prueba" anteriormente mencionada las carpetas "A", "D", "Dict", "Graph", "P" y el archivo "path_objective_Cob", los cuales se van actualizando en tiempo real según la velocidad de procesamiento de la computadora.
OJO esto quiere decir que processing y python son ASINCRÓNICOS.

El programa "análisis_datos.py" accede a las carpetas "1_robot", "2_robot" y "3_robot" mencionadas anteriormente y, asimismo, accede a cada carpeta "5-1", "5-2", ..., "15-13" y toma las matrices de cada prueba para realizar dos informes estadísticos, uno llamado "Principal", el cual contiene las estadísticas descriptivas en cuartiles, y "Resumen", el cual contiene un resumen o promedio de "Principal".

En la carpeta "Procesamiento offline" se encuentran distintos programas según la necesidad, pero en general, el único necesario es "main OFFLINE.py", el cual genera lo mismo que el ONLINE. Los demás se utilizaron una vez, no deberían ser necesarios, pero igual se dejaron por si acaso.

Los demás archivos son dependencias para el main, los cuales no se deben tocar y no es importante explicarlos.
