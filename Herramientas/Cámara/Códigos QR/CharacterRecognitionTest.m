%Con estas pruebas se utiliza la función de reconocimiento de caracteres
%de matlab, la cual trata de identificar caracteres en una imagen. Esta
%imagen es binarizada dentro de la función. Rotaciones de +- 10 grados
%deben corregirse para mejores resultados.
%Una letra minúsula, por ejemplo x debe tener al menos 20 píxeles en x o y.

probe   = imread('characterdetectionprobe.jpg');
ocrResults     = ocr(probe)
recognizedText = ocrResults.Text;    
figure;
imshow(probe);
text(600, 150, recognizedText, 'BackgroundColor', [1 1 1]);

%prueba con imagen de matlab de la documentación: prueba de que
%funciona la función ocr (optimal character recognition)
probe2   = imread('MatlabProbe.png');
ocrResults     = ocr(probe2)
recognizedText = ocrResults.Text;    
figure;
imshow(probe2);
text(600, 150, recognizedText, 'BackgroundColor', [1 1 1]);


%Prueba con la imagen del aula en formato png en lugar de jpg
%Solo detectó la R de PROE.
probe3   = imread('characterdetectionprobePNG.png');
ocrResults     = ocr(probe3)
recognizedText = ocrResults.Text;    
figure;
imshow(probe3);
text(600, 150, recognizedText, 'BackgroundColor', [1 1 1]);

%Prueba con la imagen del aula con 3 robots en formato jpg
%No reconoció las letras A ni AB.
probe4   = imread('characterdetection.jpg');
ocrResults     = ocr(probe4)
recognizedText = ocrResults.Text;    
figure;
imshow(probe4);
%el array de BackgroundColor cambia el color de fondo, admite
%valores entre 0 y 1 cada componente.
text(600, 150, recognizedText, 'BackgroundColor', [0.5 1 0.2]);


%Prueba con la imagen del aula con 3 robots en formato png
%No reconoció las letras A ni AB.
probe5   = imread('characterdetection2.png');
ocrResults     = ocr(probe5)
recognizedText = ocrResults.Text;    
figure;
imshow(probe5);
text(600, 150, recognizedText, 'BackgroundColor', [0.5 0.8 0.2]);