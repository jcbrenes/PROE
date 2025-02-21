import os
import csv

# Constants for square size and number of squares
SQUARE_SIZE = 20
RECT_WIDTH = 22
NUM_SQUARES_X = 24
NUM_SQUARES_Y = 24

def read_boolean_matrix_from_csv(filepath):
    """Reads a boolean matrix from a CSV file."""
    with open(filepath, 'r', newline='') as csvfile:
        csvreader = csv.reader(csvfile)
        matrix = [[bool(int(cell)) for cell in row] for row in csvreader]
    return matrix

def save_float_matrix_to_csv(matrix, filepath):
    """Saves a float matrix to a CSV file."""
    os.makedirs(os.path.dirname(filepath), exist_ok=True)  # Ensure the directory exists
    with open(filepath, 'w', newline='') as csvfile:
        csvwriter = csv.writer(csvfile)
        for row in matrix:
            csvwriter.writerow(row)

def create_divided_color_matrix(reduced_color_matrix):
    """Creates a divided color matrix from a reduced color matrix."""
    height = len(reduced_color_matrix)
    width = len(reduced_color_matrix[0])
    divided_color_matrix = [[0.0 for _ in range(NUM_SQUARES_X)] for _ in range(NUM_SQUARES_Y)]

    for filaDividida in range(NUM_SQUARES_Y):
        for columnaDividida in range(NUM_SQUARES_X):
            sumatoriaUnos = 0
            for filaPixeles in range(filaDividida * SQUARE_SIZE, min((filaDividida + 1) * SQUARE_SIZE, height)):
                for columnaPixeles in range(columnaDividida * RECT_WIDTH, min((columnaDividida + 1) * RECT_WIDTH, width)):
                    if reduced_color_matrix[filaPixeles][columnaPixeles]:
                        sumatoriaUnos += 1
            promedioUnos = float(sumatoriaUnos) / (SQUARE_SIZE * RECT_WIDTH)
            divided_color_matrix[filaDividida][columnaDividida] = promedioUnos

    return divided_color_matrix

# Define base path
raiz_path = "C:/Users/Isaac/Desktop/Github Repos/PROE/Optimizacion de rutas/Processing/mapeo/"

# Iterate through existing reduced color matrices
for cantidadRobots in range(1, 4):
    for tiempoPrueba in [5, 10, 15]:
        for prueba in range(1, 11):
            # Construct paths for reduced and divided matrices
            reduced_matrix_dir = f"{raiz_path}{cantidadRobots}_robot/{tiempoPrueba}-{prueba}/reduced_color_matrix/"
            divided_matrix_dir = f"{raiz_path}{cantidadRobots}_robot/{tiempoPrueba}-{prueba}/divided_color_matrix/"
            
            if os.path.exists(divided_matrix_dir):
                print("\nLa prueba ya existe.")
            
            # List all reduced matrix files in the directory
            elif os.path.exists(reduced_matrix_dir):
                for filename in os.listdir(reduced_matrix_dir):
                    if filename.startswith("reduced_color_matrix_") and filename.endswith(".csv"):
                        # Extract the date part from the filename
                        date_part = filename[len("reduced_color_matrix_"):-len(".csv")]
                        
                        reduced_matrix_test_path = os.path.join(reduced_matrix_dir, filename)
                        divided_matrix_output_path = os.path.join(divided_matrix_dir, f"divided_color_matrix_{date_part}.csv")

                        print(f"\nProcessing: {reduced_matrix_test_path}")
                        
                        # Read the reduced color matrix
                        reduced_color_matrix = read_boolean_matrix_from_csv(reduced_matrix_test_path)
                        
                        # Create the divided color matrix
                        divided_color_matrix = create_divided_color_matrix(reduced_color_matrix)
                        
                        # Save the divided color matrix
                        save_float_matrix_to_csv(divided_color_matrix, divided_matrix_output_path)
                        print(f"\nSaved: {divided_matrix_output_path}")
            else:
                print(f"\nDirectory not found: {reduced_matrix_dir}")
