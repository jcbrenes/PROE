import ADPDic_Data
import os
import re

def get_filtered_files_sorted_by_modified_time(directory, pattern):
    # Compile the regex pattern
    regex = re.compile(pattern)
    
    # List all files in the given directory and filter them using the regex pattern
    files = [os.path.join(directory, f) for f in os.listdir(directory) if os.path.isfile(os.path.join(directory, f)) and regex.match(f)]
    
    # Get the last modified time for each file and sort them
    files.sort(key=lambda x: os.path.getmtime(x))
    
    return files

file_pattern = r'^divided_color_matrix_\d{2}-\d{2}-\d{2}\.csv$'  # Regex pattern for the desired files
#raiz_path = '../Processing/mapeo/'

# Aqui es donde se pone la carpeta específica donde están TODAS las pruebas
raiz_path = "C:/Users/Isaac/Desktop/Github Repos/PROE/Optimizacion de rutas/Processing/mapeo/"
DataDang = 'MatrizPeligro.csv'

# Primer loop para pasar por las pruebas de 1, 2 y 3 robots
for cantidadRobots in range(1,4):
    cantidadRobots_path = raiz_path + str(cantidadRobots) + "_robot/"
    
    # Segundo loop para pasar por las pruebas de 5, 10 y 15 minutos
    for tiempoPrueba in [5, 10, 15]:
        
        # Tercer loop para pasar por cada prueba individual (de la primera a la décima)
        for prueba in range(1,11):
            final_path = cantidadRobots_path + str(tiempoPrueba) + "-" + str(prueba)
            
            # Intentamos recuperar todos los archivos de pruebas
            try:
                sorted_filtered_files = get_filtered_files_sorted_by_modified_time(final_path, file_pattern)
                
                # Cuarto loop para revisar cada matriz de la prueba
                for DataFile in sorted_filtered_files:
                    
                    # Extraigo la fecha de la prueba para agregársela a los archivos de optimización
                    filename = os.path.basename(DataFile)  # Get the filename from the full path
                    match = re.search(r'^divided_color_matrix_(\d{2}-\d{2}-\d{2})\.csv$', filename)                
                    if match:    
                        fecha_prueba = match.group(1)
                        archivo_A = final_path +"/A_" + fecha_prueba + ".csv"                        
                        archivo_D = final_path +"/D_" + fecha_prueba + ".csv"
                        archivo_P = final_path +"/P_" + fecha_prueba + ".csv"
                        archivo_Dict = final_path +"/Dict_" + fecha_prueba + ".json"
                        archivo_Graph = final_path +"/Graph_" + fecha_prueba + ".png"
                        
                        # Ahora sí, se hace la optimización
                        print("Optimizando archivo " + DataFile + "...\n")
                        [A,D,P,Net,F,C,Cob] = ADPDic_Data.GenData_ADPDic(DataFile,DataDang,archivo_A,archivo_D,archivo_P,archivo_Dict);
                        
                        try:
                            path_Opt = ADPDic_Data.Graph_Opt(A,Net,F,C,DataDang,True,archivo_Graph,False,0);
                        
                        except ValueError as error:
                            print(error)
                            pass
                
            
            # Si la prueba no existe (no se ha realizado) se la salta
            except FileNotFoundError as error:
                print(error)
                pass             

