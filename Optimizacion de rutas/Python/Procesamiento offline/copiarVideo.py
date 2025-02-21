import os
import shutil

# Function to find the video file
def find_video_file(directory):
    for file in os.listdir(directory):
        if file.endswith('.mkv'):
            return os.path.join(directory, file)
    return None

# Aqui es donde se pone la carpeta específica donde están TODAS las pruebas
raiz_path = "C:/Users/Isaac/Desktop/Github Repos/PROE/Optimizacion de rutas/Processing/mapeo/"
originales_path = raiz_path + "Pruebas originales/"

# Se itera por cada cantidad de robots en la prueba
for cantidadRobots in range(1, 4):
    cantidadRobots_original_path = originales_path + str(cantidadRobots) + "_robot/"
    cantidadRobots_path = raiz_path + str(cantidadRobots) + "_robot/"

    # Se itera por cada tiempo de prueba
    for tiempoPrueba in [5, 10, 15]:
        # Se itera por cada número de prueba
        for prueba in range(1, 14):
        
            dest_dir = cantidadRobots_path + str(tiempoPrueba) + "-" + str(prueba)
            source_dir = cantidadRobots_original_path + str(tiempoPrueba) + "-" + str(prueba)
            
            # Intentamos recuperar todos los archivos de pruebas
            try:
                # Find and copy the video file
                video_file = find_video_file(source_dir)
                if video_file:
                    try:
                        shutil.copy(video_file, dest_dir)
                        print(f"\nVideo file '{os.path.basename(video_file)}' successfully copied to '{dest_dir}'")
                    except Exception as e:
                        print(f"\nError while copying the file: {e}")
                else:
                    print("\nNo video file found.")
                
            # Si la prueba no existe (no se ha realizado) se la salta
            except FileNotFoundError:
                print(f"\nLa prueba {cantidadRobots}_robot/{tiempoPrueba}-{prueba} no existe.")
                pass
