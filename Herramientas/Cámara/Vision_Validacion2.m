%--------------Etapa de captura de imágenes
clear cam;
tamanoCuadroMedicion=40;
for img= 1:CantidadEjemplosCaptura
    cam=webcam(2);%Selecciona la camara USB
    cam.Resolution='640x480'; %Es la resolución máxima de la cámara (DroidCam)
    h = msgbox('Capture Imagen');
    for j=1:2
        filename=strcat('ejemplo',num2str(img),'(',num2str(j),').jpg');
        %strcat concatena caracteres
        preview(cam);
        pause;
        b=snapshot(cam);
        imwrite(b,filename);
        h = msgbox(strcat('ejemplo',num2str(img),'(',num2str(j),').jpg'));
        %msgbox crea una caja de mensaje
    end
    closePreview(cam);
    clear cam;
    
    %ejemplo corresponde al nombre de la imagen, carga el ejemplo 1(1).jpg y el
    %ejemplo 1(2).jpg.

    Im1=imread(strcat('ejemplo',num2str(img),'(1).jpg'));
    %Requiere la conversión del espacio de color rgb que corresponde a 3
    %matrices, una del color rojo, una del verde y otra del azul a una sola
    %matriz en intensidad de grises.
   
    Im2=imread(strcat('ejemplo',num2str(img),'(2).jpg'));
    %Leer la imagen a medir
    imOrig = Im1;

    %Se realiza la conversión RGB to Grayscale de "im"
    Im1GRIS=rgb2gray(Im1);
    %Se realiza la conversión binaria de la imagen. Ahora solo habrá negro
    %o blanco, según el umbral definido
    Im1BN=im2bw(Im1GRIS,0.6);%0.X corresponde al umbral para el aislamiento entre las zonas de interés y el fondo.
     
    %De aquí en adelante se trabaja con Im1 (sin quitar distorision)
    [imagePoints, boardSize] = detectCheckerboardPoints(Im1);
    worldPoints = generateCheckerboardPoints(boardSize, tamanoCuadroMedicion);
    %R es la matriz de rotacion y t el vector de traslacion.
    %Permite transformar puntos del sistema coordenado del mundo
    %hasta el sistema coordenado de la camara.
    [R, t] = extrinsics(imagePoints, worldPoints, params);
    worldPoints1 = pointsToWorld(params, R, t, imagePoints);
    d = worldPoints1(2, :) - worldPoints1(1, :);
    mmCuadroPatron=hypot(d(1), d(2))
    d=undistortedPoints(2,:)-undistortedPoints(1,:)
    pixelesCuadroPatron=hypot(d(1),d(2))
    %Se toma como referencia la medida de uno de los cuadros del patrón de
    %calibración de la imagen para extraer la conversión de unidades de la
    %imagen a unidades reales.
    mmPorPixel=mmCuadroPatron/pixelesCuadroPatron
    %ya se tiene almacenado la conversion correspondiente en mm/pixel
    
    imOrig = Im2;

    points = detectCheckerboardPoints(imOrig);
    [undistortedPoints,reprojectionErrors] = undistortPoints(points, params);
    [im, newOrigin] = undistortImage(imOrig, params, 'OutputView', 'full');
    undistortedPoints = [undistortedPoints(:,1) - newOrigin(1), undistortedPoints(:,2) - newOrigin(2)];
    Im2GRIS=rgb2gray(im);
    Im2BN=im2bw(Im2GRIS,0.7);
    ImSuperpuesta=Im1BN&Im2BN;
    %Se superponen las dos imágenes con  la segmentación del patrón
    %utilizado para determinar la orientación de los robots.

    %---Parámetros de detección del patrón----------------------- 
    cantidadCirculos=1; %Parámetro utilizado en las funciones de extracción de círculos
    Rmin=20;%usado para buscar los círculos pequeños, variar en caso de no detección
    Rmax=30;%usado en el círculo pequeño
    Rmin2=30;%utilizado para determinar el círculo grande
    Rmax2=50;%utilizado para el círculo grande
    RminRef=90;% utilizados para la detección del sistema de coordenadas.
    RmaxRef=110;
    
    %---Extracción de los círculos----------------------------
    %Se está utilizando la funcion de Hough circular con la función
    %imfindcircles,permite la detección de círculos oscuros mediante
    %ObjectPolarity.
    %Se presenta la primera parte para la detección de los círculos pequeños,
    %las respuestas se almacenan en centroPequeño y radioPequeño.
    [dimensionY dimensionX]=size(Im1(:,:,1));
    [centers, radii] = imfindcircles(Im1,[Rmin Rmax],'ObjectPolarity','dark');
    centroPequeno(1,:) = centers(1:cantidadCirculos,:);
    radioPequeno(1) = radii(1:cantidadCirculos); 
    [centers, radii] = imfindcircles(Im2,[Rmin Rmax],'ObjectPolarity','dark');
    centroPequeno(2,:) = centers(1:cantidadCirculos,:);
    radioPequeno(2) = radii(1:cantidadCirculos);
    d1=figure;
    imshow(ImSuperpuesta);
    hold on


    %Se realiza la búsqueda de los círculos grandes y los resultados se
    %almancenan en centroGrande y radioGrande.
    viscircles(centroPequeno, radioPequeno,'EdgeColor','r');
    plot(centers(:,1),centers(:,2),'rx')
    [centers, radii] = imfindcircles(Im1,[Rmin2 Rmax2],'ObjectPolarity','dark');
    centroGrande(1,:) = centers(1:cantidadCirculos,:);
    radioGrande(1) = radii(1:cantidadCirculos); 
    [centers, radii] = imfindcircles(Im2,[Rmin2 Rmax2],'ObjectPolarity','dark');
    centroGrande(2,:) = centers(1:cantidadCirculos,:);
    radioGrande(2) = radii(1:cantidadCirculos); 
    viscircles(centroGrande, radioGrande,'EdgeColor','b');
    plot(centers(:,1),centers(:,2),'bx')

    %----Muestra de resultados de forma gráfica-------------
    lineasCarro1=[centroGrande(1,:);centroPequeno(1,:)];
    lineasCarro2=[centroGrande(2,:);centroPequeno(2,:)];
    plot(lineasCarro1(:,1),lineasCarro1(:,2),'LineWidth',2,'Color','red');
    plot(lineasCarro2(:,1),lineasCarro2(:,2),'LineWidth',2,'Color','blue');


    %---Construcción de vectores para determinar orientación y distancia de
    %separación
    ab1 =[lineasCarro1(1,1)-lineasCarro1(2,1) -(lineasCarro1(1,2)-lineasCarro1(2,2))]; 
    ab2 = [lineasCarro2(1,1)-lineasCarro2(2,1) -(lineasCarro2(1,2)-lineasCarro2(2,2))]; 
    vect1 = ab1;%Vector del agente antes de inicial su movimiento
    vect2 = ab2;%Vector del agente al finalizar su movimiento
    vectRef=[100 0];%Vector horizontal en la imagen
    %Se requiere determinar el ángulo de separación entre la orientación
    %inicial y la orientación final, para esto, se referencian ambos
    %vectores respecto al eje horizontal de la imagen
    %atan2 retorna la tangente inversa en el cuarto cuadrante de Y y X
    citaInicial=atan2(vect1(2),vect1(1))*180/pi; %cita_inicial en grados
    if(citaInicial<=0)
        citaInicial=360+citaInicial;
    end
    citaFinal=atan2(vect2(2),vect2(1))*180/pi;%cita_final
    if(citaFinal<=0)
        citaFinal=360+citaFinal;
    end
    deltaCita = citaFinal-citaInicial%delta_cita es el error en la orientación
    %norm retorna la norma euclídea de un vector
    distancia=norm((centroGrande(1,:)-centroGrande(2,:))); %desplazamiento en pixeles del punto de partida y el punto final
    r_exp=distancia*mmPorPixel %r_experimental es el desplazamiento en mm
    deltaX=(centroGrande(1,2)-centroGrande(1,1))*mmPorPixel;%deltaX del desplazamiento experimental, con la imagen
    deltaY=(centroGrande(2,2)-centroGrande(2,1))*mmPorPixel;%deltay del desplazamiento exp, con la imagen
    betaExp=atan2(deltaY/deltaX)*180/pi;%angulo del desplazamiento experimental, referenciado con la imagen
    betaTeorico=anguloVector1;% se rota el sistema para que coincida con el del agente
    deltaBeta=betaExp-betaTeorico;%diferencia entre el desplazamiento teórico y el experimental
    deltaX2=r_exp*cos(deltaBeta)*180/pi;%proyección x del r_exp sobre el desplazamiento teórico
    deltaY2=r_exp*sin(deltaBeta)*180/pi;%proyección y del r_exp sobre el desplazamiento teórico
    %---Almacenamiento de resultados, la separación es en mm y el ángulo en
    %grados.
    result=strcat(num2str(deltaX2),';',num2str(deltaY2),';',num2str(deltaCita))
    cont=cont+1;
end