import os
import re
import csv

def get_filtered_files_sorted_by_modified_time(directory, pattern):
    # Compile the regex pattern
    regex = re.compile(pattern)
    
    # List all files in the given directory and filter them using the regex pattern
    files = [os.path.join(directory, f) for f in os.listdir(directory) if os.path.isfile(os.path.join(directory, f)) and regex.match(f)]
    
    # Get the last modified time for each file and sort them
    files.sort(key=lambda x: os.path.getmtime(x))
    
    return files

file_pattern = r'^color_matrix_\d{2}-\d{2}-\d{2}\.csv$'  # Regex pattern for the desired files

# Aqui es donde se pone la carpeta específica donde están TODAS las pruebas
raiz_path = "C:/Users/Isaac/Desktop/Github Repos/PROE/Optimizacion de rutas/Processing/mapeo/"
originales_path = raiz_path + "Pruebas originales/"
DataDang = 'MatrizPeligro.csv'

# Se itera por cada cantidad de robots en la prueba
for cantidadRobots in range(1, 4):
    cantidadRobots_original_path = originales_path + str(cantidadRobots) + "_robot/"
    cantidadRobots_path = raiz_path + str(cantidadRobots) + "_robot/"

    # Se itera por cada tiempo de prueba
    for tiempoPrueba in [5, 10, 15]:
        
        # Se itera por cada número de prueba, de la 10 en adelante ya están reducidas
        for prueba in range(1, 11):
        
            final_path = cantidadRobots_path + str(tiempoPrueba) + "-" + str(prueba)
            final_original_path = cantidadRobots_original_path + str(tiempoPrueba) + "-" + str(prueba)
            color_matrix_path = final_original_path + "/color_matrix/"
            divided_matrix_path = final_path + "/divided_color_matrix/"
            reduced_matrix_path = final_path + "/reduced_color_matrix/"
              
            # Ensure reduced_matrix_path directory exists
            if not os.path.exists(reduced_matrix_path):
                os.makedirs(reduced_matrix_path)
            
            # Intentamos recuperar todos los archivos de pruebas
            try:
                sorted_filtered_files = get_filtered_files_sorted_by_modified_time(color_matrix_path, file_pattern)
                    
                # Loop para revisar cada matriz de la prueba
                for DataFile in sorted_filtered_files:
                    
                    # Extraigo la fecha de la prueba para agregársela a los archivos de optimización
                    filename = os.path.basename(DataFile)  # Get the filename from the full path
                    match = re.search(r'^color_matrix_(\d{2}-\d{2}-\d{2})\.csv$', filename)                
                    if match:    
                        fecha_prueba = match.group(1)
                        
                        # Abrir el archivo CSV y cargarlo en una matriz 2D
                        with open(DataFile, newline='') as csvfile:
                            csvreader = csv.reader(csvfile)
                            color_matrix = [row for row in csvreader]
                            
                        # Creo la matriz reducida
                        reduced_color_matrix = [row[16:-96] for row in color_matrix]
                        
                        reduced_matrix_test_path = os.path.join(reduced_matrix_path, "reduced_color_matrix_" + fecha_prueba + ".csv")
                        
                        # Write the reduced matrix to the CSV file
                        try:
                            with open(reduced_matrix_test_path, 'w', newline='', encoding='utf-8') as csvfile:
                                csv_writer = csv.writer(csvfile)
                                csv_writer.writerows(reduced_color_matrix)
                            
                            print("\nCreando archivo " + reduced_matrix_test_path + "...\n")
                        
                        except ValueError as error:
                            print(error, "\n")
                            pass
                
            # Si la prueba no existe (no se ha realizado) se la salta
            except FileNotFoundError:
                print(f"\nLa prueba {cantidadRobots}_robot/{tiempoPrueba}-{prueba} no existe.")
                pass
