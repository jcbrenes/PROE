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
RminPequeno=round((RadioPequeno/mmPorPixel)*0.9)
RmaxPequeno=round((RadioPequeno/mmPorPixel)*1.5)

%Ahora se hace el mismo procedimiento para el círculo grande.
RminGrande=round((RadioGrande/mmPorPixel)*1)
RmaxGrande=round((RadioGrande/mmPorPixel)*1.5)

%Prueba de búsqueda círculo pequeño
%Dark se refiere a que el círculo es oscuro sobre un fondo claro.

%Prueba 02 febrero 2020 INICIO (requiere su workspace)
%Mejor combinación hasta ahora de multiplicadores: 1 y 2, pero detecta los
%grandes.
RminPequeno=round((RadioPequeno/mmPorPixel)*0.84)
RmaxPequeno=round((RadioPequeno/mmPorPixel)*1.7)
A = ImSuperpuesta; imshow(A)
[centersDarkp, radiiDarkp] = imfindcircles(A,[RminPequeno RmaxPequeno],'ObjectPolarity','dark');
viscircles(centersDarkp, radiiDarkp,'EdgeColor','b');

%Prueba de búsqueda círculo grande
[centersDarkg, radiiDarkg] = imfindcircles(A,[RminGrande RmaxGrande],'ObjectPolarity','dark');
viscircles(centersDarkg, radiiDarkg,'EdgeColor','r');
%FIN

A = imread('ejemplo1(1).jpg'); imshow(A)
[centersDarkp, radiiDarkp] = imfindcircles(A,[RminPequeno RmaxPequeno],'ObjectPolarity','dark');
viscircles(centersDarkp, radiiDarkp,'EdgeColor','b');

%Prueba de búsqueda círculo grande

B = imread('ejemplo1(1).jpg'); imshow(B)
[centersDarkg, radiiDarkg] = imfindcircles(B,[RminGrande RmaxGrande],'ObjectPolarity','dark');
viscircles(centersDarkg, radiiDarkg,'EdgeColor','r');

%Prueba con el nuevo identificador de círculos inscritos de mayor tamaño
%El radio del círculo pequeño impreso (Imagen llamada "Identificador Modificado"), 
%es de 30mm. El radio del círculo grande es de 50mm. Por lo tanto, con los
%cálculos siguientes se desea aproximar a cuántos píxeles equivalen estos
%radios.
%Ahora son el doble de grandes en comparación al primer identificador
%Otra diferencia es que el círculo grande está sobre fondo oscuro,
%mientras que el círculo pequeño está sobre fondo claro. La línea 
%que se puede trazar entre sus centros marcará la orientación del robot.
RadioPequeno=30; %15mm es el radio del círculo pequeño
RadioGrande=50; %25mm es el radio del círculo grande

%El rango de radios pequeños se establece como el radio calculado en
%píxeles ± un porcentaje (debe ajustarce de ser necesario). Se utiliza la función round para redondear el
%resultado al entero más cercano, ya que no pueden existir fracciones de
%píxeles.
RminPequeno=round((RadioPequeno/mmPorPixel)*0.6)%resultado cercano a 6 píxeles
RmaxPequeno=round((RadioPequeno/mmPorPixel)*1.4)%resultado cercano a 15 píxeles

%Ahora se hace el mismo procedimiento para el círculo grande.
RminGrande=round((RadioGrande/mmPorPixel)*0.85)
RmaxGrande=round((RadioGrande/mmPorPixel)*1.65)

%Prueba de búsqueda círculo pequeño
A = imread('ejemplo1(1)Mod.jpg'); imshow(A)
%Ojo: debe buscar círculos oscuros
[centersDarkp, radiiDarkp] = imfindcircles(A,[RminPequeno RmaxPequeno],'ObjectPolarity','dark');
viscircles(centersDarkp, radiiDarkp,'EdgeColor','b');%lo marca en color azul

%Prueba de búsqueda círculo grande
%Ojo: debe buscar círculo claros
B = imread('ejemplo1(1)Mod.jpg'); imshow(B)
[centersDarkg, radiiDarkg] = imfindcircles(B,[RminGrande RmaxGrande],'ObjectPolarity','bright');
viscircles(centersDarkg, radiiDarkg,'EdgeColor','r');%lo marca en color rojo

%Ahora se prueba con la imagen ejemplo1(2)Mod para comprobar que sirve la
%detección de los círculos al tener otra orientación y posición.
%Se observa que ya no hay problema con las manchas del piso.
%Prueba de búsqueda círculo pequeño
C = imread('ejemplo1(2)Mod.jpg'); imshow(C)
%Ojo: debe buscar círculos oscuros
[centersDarkp, radiiDarkp] = imfindcircles(C,[RminPequeno RmaxPequeno],'ObjectPolarity','dark');
viscircles(centersDarkp, radiiDarkp,'EdgeColor','b');%lo marca en color azul

%Prueba de búsqueda círculo grande
%Ojo: debe buscar círculo claros
D = imread('ejemplo1(2)Mod.jpg'); imshow(D)
[centersDarkg, radiiDarkg] = imfindcircles(D,[RminGrande RmaxGrande],'ObjectPolarity','bright');
viscircles(centersDarkg, radiiDarkg,'EdgeColor','r');%lo marca en color rojo