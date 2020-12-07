


%Adicionalmente, se realiza una calibración de la cámara para eliminar
%distorsion provocado por el lente, para mayor referencia : https://www.mathworks.com/help/vision/ug/camera-calibration.html
%Para ser capaces de manejar los datos de la webcam es necesario importar
%la biblioteca de webcam. Se descarga desde el gestor de bibliotecas.
%Home/Add-Ons y buscar webcam.
%
    
%----------Paso1: Abrir en APPS Camera Calibrator-------------------
% Camera Calibrator se encuentra en la pestaña APPS->Image Proccessing
% Es necesario lograr una cantidad de ejemplos útiles para que
% la aplicación determine los parámetros correspondientes. 

%Si se tiene cámara web usb, y no se han tomado las imágenes requeridas:

% 1.a: Seleccionar en Calibration/AddImages/FromCamera
% 1.b: Seleccionar en CAMERA/Camera/ HD Pro Webcam C920
% 1.c: Cambiar la resolución a la más alta en Camera Properties, y dejar lo
% demás en auto
% 1.d: Para iniciar la captura ajustar los valores de Capture Interval y el
% número de imágenes a capturar. Finalmente dar clic en Capture.
%Nota: Se debe tratar que el patrón de calibración esté en posiciones
%diferentes y en orientaciones diferentes sobre el plano de trabajo,
%adicionalmente agregar un par de ejemplos un poco inclinados para que
%permita calibrar correctamente. Guardar en la carpeta del programa

% Si ya obtuvo, mediante el código CalibracionCamaraP1.m las imágenes requeridas:

% Seleccionar Calibration/AddImages y seleccione el conjunto capturado
% Si no aparecen las imágenes tomadas debe abrir el explorador de archivos
% y cambiar su nombre con la extensión .png
% El lado de cada cuadrado de calibración es de 40 mm
% Una vez se logra un set de imágenes útiles para la calibración, se
% deben eliminar los ejemplos que fueron rechazados.
% Seleccionar el set de imágenes y Renombrar los archivos como Image,
% deberán aparecer los archivos como Image(1).png, Image(2).png...
% El número de imagen se pone entre paréntesis para diferenciar las
% imágenes aceptadas, de todas las que fueron tomadas.
% Cambiar numImages a la cantidad de imágenes útiles para calibración.

%ANTES DE CORRER ESTE CÓDIGO:

%SI TIENE HABILITADA LA OPCIÓN, DESDE LA APLICACIÓN DE CAMERA
%CALIBRATOR, DE PULSAR EL BOTÓN "Calibrate", hágalo y espere, al finalizar la
%operación, exportar parámetros llamándolos params. Al hacer esto, se deben
%obtener los mismos resultados que al correr este código.
%Si realiza la calibración con el Botón Calibrate, solamente debe ejecutar
%las líneas 64 hasta 68, ya que contienen variables que se utilizarán más
%adelante en Vision_Validacion.m

cont=1;
numImages = 10; %Cantidad de imagenes aceptadas para calibración
CantidadEjemplosCaptura=1;%Variable para modificar la cantidad de ejemplos que se almacenan en los resultados
squareSize = 40; % Tamaño del lado del cuadro en el patrón de calibración, en mm
tamanoCuadroMedicion=15; %aun no se encuentra por qué 15
%--------Proceso de Calibración-----------------------
%Se importan las imagenes de calibración que fueron previamente almacenadas
%Se debe modificar la extensión de búsqueda con la dirección de la carpeta
%donde están las imágenes.

%Aquí se cambia la dirección de las imágenes según la computadora con la
%que se trabaje.

%Dirección Kevin
%extensionImCalibracion='C:\Users\Kevin Morales\Documents\GitHub\PROE\Herramientas\Cámara\';

%Dirección Cindy
extensionImCalibracion='C:\Users\ccalderon\OneDrive - TEC\GitHub\PROE\Herramientas\Cámara\';
%cell genera una matriz de celdas, en este caso, de 1xnumImages
files = cell(1, numImages);
%fullfile retorna un vector con el directorio de un archivo
%strcat concatena cadenas horizontalmente
for i = 1:numImages
    files{i} = fullfile(strcat(extensionImCalibracion,'Image(',num2str(i),').png'));
end
magnification = 25; %no veo donde se usa aun.
%Se determina I como toda la fila de extensiones de las imágenes para
%calibración
I = imread(files{1});
[imagePoints, boardSize] = detectCheckerboardPoints(files);
worldPoints = generateCheckerboardPoints(boardSize, squareSize);
params = estimateCameraParameters(imagePoints, worldPoints);
