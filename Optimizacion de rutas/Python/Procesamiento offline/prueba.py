import ADPDic_Data
import os
import re
import csv
import shutil

# --- Utility Functions ---
def get_filtered_files_sorted_by_modified_time(directory, pattern):
    regex = re.compile(pattern)
    files = [os.path.join(directory, f) for f in os.listdir(directory) if os.path.isfile(os.path.join(directory, f)) and regex.match(f)]
    files.sort(key=lambda x: os.path.getmtime(x))
    return files

def read_boolean_matrix_from_csv(filepath):
    with open(filepath, 'r', newline='') as csvfile:
        csvreader = csv.reader(csvfile)
        return [[bool(int(cell)) for cell in row] for row in csvreader]

def save_float_matrix_to_csv(matrix, filepath):
    os.makedirs(os.path.dirname(filepath), exist_ok=True)
    with open(filepath, 'w', newline='') as csvfile:
        csvwriter = csv.writer(csvfile)
        for row in matrix:
            csvwriter.writerow(row)

def find_video_file(directory):
    for file in os.listdir(directory):
        if file.endswith('.mkv'):
            return os.path.join(directory, file)
    return None


# --- Matrix Processing Functions ---
def process_color_matrix(input_path, output_path):
    """Reduce the color matrix and save it to the output path."""
    file_pattern = r'^color_matrix_\d{2}-\d{2}-\d{2}\.csv$'
    sorted_filtered_files = get_filtered_files_sorted_by_modified_time(input_path, file_pattern)
    
    for DataFile in sorted_filtered_files:
        filename = os.path.basename(DataFile)
        match = re.search(r'^color_matrix_(\d{2}-\d{2}-\d{2})\.csv$', filename)
        if match:
            fecha_prueba = match.group(1)
            with open(DataFile, newline='') as csvfile:
                color_matrix = [row for row in csv.reader(csvfile)]
            reduced_color_matrix = [row[16:-96] for row in color_matrix]
            reduced_matrix_test_path = os.path.join(output_path, f"reduced_color_matrix_{fecha_prueba}.csv")
            
            if os.path.exists(reduced_matrix_test_path):
                print(f"\nReduced color matrix already exists: {reduced_matrix_test_path}")
                continue  # Skip this file if the reduced matrix already exists
            
            save_float_matrix_to_csv(reduced_color_matrix, reduced_matrix_test_path)

def calculate_divided_color_matrix(reduced_matrix_file, output_file):
    """
    This function takes the reduced color matrix file and divides it into smaller sections,
    then writes the divided matrix to a new file.
    
    :param reduced_matrix_file: Path to the reduced color matrix file.
    :param output_file: Path where the divided color matrix will be saved.
    """
    try:
        # Load the reduced color matrix
        with open(reduced_matrix_file, 'r') as f:
            reader = csv.reader(f)
            reduced_matrix = list(reader)

        # Logic to divide the matrix (for demonstration purposes, 
        # we assume it divides the matrix into 4 equal parts)
        divided_matrix = []
        rows = len(reduced_matrix)
        cols = len(reduced_matrix[0]) if rows > 0 else 0
        
        # Example of dividing the matrix into quadrants (this is just a simple division logic)
        half_rows = rows // 2
        half_cols = cols // 2
        
        for i in range(0, rows, half_rows):
            for j in range(0, cols, half_cols):
                # Extract the sub-matrix
                sub_matrix = [row[j:j + half_cols] for row in reduced_matrix[i:i + half_rows]]
                divided_matrix.append(sub_matrix)

        # Save the divided matrix to the output file
        with open(output_file, 'w', newline='') as f_out:
            writer = csv.writer(f_out)
            for sub_matrix in divided_matrix:
                for row in sub_matrix:
                    writer.writerow(row)
                writer.writerow([])  # Blank row to separate divided matrices
        
        print(f"Divided matrix saved to: {output_file}")

    except Exception as e:
        print(f"\nError processing file {reduced_matrix_file}: {e}")


def create_divided_color_matrix(reduced_matrix_path, output_path):
    """Divide the reduced color matrix into smaller matrices and save them."""
    for filename in os.listdir(reduced_matrix_path):
        if filename.startswith("reduced_color_matrix_") and filename.endswith(".csv"):
            date_part = filename[len("reduced_color_matrix_"):-len(".csv")]
            reduced_matrix_test_path = os.path.join(reduced_matrix_path, filename)
            divided_matrix_output_path = os.path.join(output_path, f"divided_color_matrix_{date_part}.csv")
            
            if os.path.exists(divided_matrix_output_path):
                print(f"\nDivided color matrix already exists: {divided_matrix_output_path}")
                continue  # Skip this file if the divided matrix already exists
            
            reduced_color_matrix = read_boolean_matrix_from_csv(reduced_matrix_test_path)
            divided_color_matrix = calculate_divided_color_matrix(reduced_color_matrix)
            save_float_matrix_to_csv(divided_color_matrix, divided_matrix_output_path)


# --- Optimization Functions ---
def optimize_color_matrix(input_path, output_path, DataDang):
    
    try:
        # Creo carpetas para cada tipo de archivo
        os.mkdir(output_path +"A/")
        os.mkdir(output_path +"D/")
        os.mkdir(output_path +"P/")
        os.mkdir(output_path +"Dict/")
        os.mkdir(output_path +"Graph/")
        
        """Optimize the divided color matrix and save results."""
        sorted_filtered_files = get_filtered_files_sorted_by_modified_time(input_path, r'^divided_color_matrix_\d{2}-\d{2}-\d{2}\.csv$')
        
        with open(output_path, 'w', newline='', encoding='utf-8') as csvfile:
            writer_path = csv.writer(csvfile)
            
            for DataFile in sorted_filtered_files:
                filename = os.path.basename(DataFile)
                match = re.search(r'^divided_color_matrix_(\d{2}-\d{2}-\d{2})\.csv$', filename)
                if match:
                    fecha_prueba = match.group(1)
                    # Simulate optimization logic here
                    print(f"\nOptimizing file {DataFile}...")
                    
                    archivo_A = output_path +"A/A_" + fecha_prueba + ".csv"                        
                    archivo_D = output_path +"D/D_" + fecha_prueba + ".csv"
                    archivo_P = output_path +"P/P_" + fecha_prueba + ".csv"
                    archivo_Dict = output_path +"Dict/Dict_" + fecha_prueba + ".json"
                    archivo_Graph = output_path +"Graph/Graph_" + fecha_prueba + ".png"
                    
                    # Ahora sí, se hace la optimización
                    print("\nOptimizando archivo " + DataFile + "...\n")
                    [A,D,P,Net,F,C,Cob] = ADPDic_Data.GenData_ADPDic(DataFile,DataDang,archivo_A,archivo_D,archivo_P,archivo_Dict);
                    
                    try:
                        path_Opt = ADPDic_Data.Graph_Opt(A,Net,F,C,DataDang,True,archivo_Graph,False,0);
                        writer_path.writerow(list(path_Opt[0]['path']) + list(path_Opt[0]['objective']) + [Cob])
                    except ValueError as error:
                        print(error,"\n\n")
                        pass
                    
                    writer_path.writerow([fecha_prueba, 'example_path', 'example_objective', 'example_Cob'])
    
    except FileExistsError:
        print(f"\nLa optimización de {input_path} ya estaba hecha.")
        pass
        

def copy_video_files(original_path, dest_path):
    """Find and copy the video file from source to destination."""
    video_file = find_video_file(original_path)
    if video_file:
        shutil.copy(video_file, dest_path)
        print(f"\nVideo file '{os.path.basename(video_file)}' copied to '{dest_path}'")
    else:
        print("\nNo video file found.")


def check_test_exists(directory):
    """Check if the test directory exists."""
    if not os.path.exists(directory):
        print(f"\nThe test directory {directory} does not exist. Skipping this test.")
        return False
    return True

def main():
    raiz_path = "C:/Users/Isaac/Desktop/Github Repos/PROE/Optimizacion de rutas/Processing/mapeo/"
    originales_path = raiz_path + "Pruebas originales/"
    DataDang = 'MatrizPeligro.csv'

    for cantidadRobots in range(1, 4):
        for tiempoPrueba in [5, 10, 15]:
            for prueba in range(1, 14):
                source_dir = f"{originales_path}{cantidadRobots}_robot/{tiempoPrueba}-{prueba}"
                final_dir = f"{raiz_path}{cantidadRobots}_robot/{tiempoPrueba}-{prueba}/"

                # Check if the test directory exists
                if not check_test_exists(source_dir):
                    continue  # Skip this test if the directory does not exist

                # Step 1: Process color matrices
                color_matrix_path = f"{source_dir}/color_matrix/"
                reduced_matrix_path = f"{final_dir}/reduced_color_matrix/"
                process_color_matrix(color_matrix_path, reduced_matrix_path)

                # Step 2: Create divided color matrices
                divided_matrix_path = f"{final_dir}/divided_color_matrix/"
                create_divided_color_matrix(reduced_matrix_path, divided_matrix_path)

                # Step 3: Copy video file
                copy_video_files(source_dir, final_dir)

                # Step 4: Perform optimization
                Cob_path = f"{final_dir}/path_objective_Cob.csv"
                
                if os.path.exists(Cob_path):
                    print(f"\nOptimization already exists: {Cob_path}")
                    continue  # Skip this test if the optimization already exists
                
                optimize_color_matrix(divided_matrix_path, Cob_path, DataDang)

if __name__ == "__main__":
    main()
