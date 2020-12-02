%Con este script se puede poner a prueba la detección de los círculos en 
%una imagen. El rango de radios es en píxels, y depende de la altura a la
%que se ubique la cámara.

%Para obtener una primera aproximación del tamaño en píxeles de estos
%radios, se utilizará la variable mmPorPixel, ya sea cargando el Workspace
%más reciente, o realizando el proceso de calibración.

%El radio del círculo pequeño impreso (PDF llamado "Vision_Guia"), es de
%15mm. El radio del círculo grande es de 25mm. Por lo tanto, con los
%cálculos siguientes se desea aproximar a cuántos píxeles equivalen estos
%radios.

RadioPequeno=15; %15mm es el radio del círculo pequeño
RadioGrande=25; %25mm es el radio del círculo grande

%El rango de radios pequeños se establece como el radio calculado en
%píxeles ± un porcentaje (debe ajustarce de ser necesario). Se utiliza la función round para redondear el
%resultado al entero más cercano, ya que no pueden existir fracciones de
%píxeles.
RminPequeno=round((RadioPequeno/mmPorPixel)*0.5)
RmaxPequeno=round((RadioPequeno/mmPorPixel)*1.5)

%Ahora se hace el mismo procedimiento para el círculo grande.
RminGrande=round((RadioGrande/mmPorPixel)*0.75)
RmaxGrande=round((RadioGrande/mmPorPixel)*1.5)

%Prueba de búsqueda círculo pequeño
%Dark se refiere a que el círculo es oscuro sobre un fondo claro.

A = imread('ejemplo1(1).jpg'); imshow(A)
[centersDarkp, radiiDarkp] = imfindcircles(A,[RminPequeno RmaxPequeno],'ObjectPolarity','dark');
viscircles(centersDarkp, radiiDarkp,'EdgeColor','b');

%Prueba de búsqueda círculo grande

B = imread('ejemplo1(1).jpg'); imshow(B)
[centersDarkg, radiiDarkg] = imfindcircles(B,[RminGrande RmaxGrande],'ObjectPolarity','dark');
viscircles(centersDarkg, radiiDarkg,'EdgeColor','r');
