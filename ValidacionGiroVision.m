%Este programa tiene como objetivo la estimación del ángulo de giro y la
%separación entre los agentes utilizando un patrón visual como referencia
%Requiere de la captura de dos imágenes, una antes del giro y otra
%posterior al mismo.

clc;
clear all;
close all;
%ejemplo corresponde al nombre de la imagen, carga el ejemplo 5(1).jpg y el
%ejemplo 5(2).jpg.
ejemplo=5;
Im1=imread(strcat(num2str(ejemplo),' (1).jpg'));
%Requiere la conversión de el espacio de color rgb que corresponde a 3
%matrices, una del color rojo, una del verde y otra del azul a una sola
%matriz en intensidad de grises.
Im1=rgb2gray(Im1);
Im1=im2bw(Im1,0.6);%0.6corresponde al umbral para el aislamiento entre las zonas de interés y el fondo.
Im2=imread(strcat(num2str(ejemplo),' (2).jpg'));
figure
Im2=rgb2gray(Im2);
Im2=im2bw(Im2,0.6);
ImSuperpuesta=Im1&Im2;
imshow(ImSuperpuesta)

%---Parámetros de detección del patrón----------------------- 
cantidadCirculos=1; %Parámetro utilizado en las funciones de extracción de círculos
Rmin=40;%usado para buscar los círculos pequeños, variar en caso de no detección
Rmax=75;%usado en el círculo pequeño
Rmin2=75;%utilizado para determinar el círculo grande
Rmax2=130;%utilizado para el círculo grande
RminRef=90;% utilizados para la detección del sistema de coordenadas.
RmaxRef=110;
cont=1;


%---Extracción de los círculos----------------------------
%Se está utilizando la funcion de Hough circular con la función
%imfindcircles,permite la detección de círculos oscuros mediante
%ObjectPolarity,
%Se presenta la primerpa parte para la detección de los círculos pequeños,
%las respuestas se almacenan en centroPequeño y radioPequeño.
[dimensionY dimensionX]=size(Im1(:,:,1));
[centers, radii] = imfindcircles(Im1,[Rmin Rmax],'ObjectPolarity','dark');
centroPequeno(1,:) = centers(1:cantidadCirculos,:);
radioPequeno(1) = radii(1:cantidadCirculos); 
[centers, radii] = imfindcircles(Im2,[Rmin Rmax],'ObjectPolarity','dark');
centroPequeno(2,:) = centers(1:cantidadCirculos,:);
radioPequeno(2) = radii(1:cantidadCirculos);
d=figure;
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
ab1 =[lineasCarro1(2,1)-lineasCarro1(1,1) lineasCarro1(2,2)-lineasCarro1(1,2)]; 
ab2 = [lineasCarro2(2,1)-lineasCarro2(1,1) lineasCarro2(2,2)-lineasCarro2(1,2)]; 
vect1 = ab1; % create a vector based on the line equation
vect2 = ab2;
dp = dot(vect1, vect2);
length1 = sqrt(sum(vect1.^2));
length2 = sqrt(sum(vect2.^2));
angulo = acos(dp/(length1*length2))*180/pi
distancia=norm((centroGrande(1,:)-centroGrande(2,:)));
separacion=(distancia*(25/radioGrande(1,1)))

%---Almacenamiento de resultados, la separación es en mm y el ángulo en
%grados.
result(cont,1)=ejemplo;
result(cont,2)=angulo;
result(cont,3)=separacion;

% try
% %---Detección de la referencia-------------------------------------
% [centers, radii] = imfindcircles(A,[RminRef RmaxRef],'ObjectPolarity','bright');
% centrosRef = centers(1:3,:);
% radiosRef = radii(1:3);
% viscircles(centrosRef, radiosRef,'EdgeColor','g');
% len1=norm((centrosRef(1,:)-centrosRef(2,:)));
% len2=norm((centrosRef(1,:)-centrosRef(3,:)));
% len3=norm((centrosRef(2,:)-centrosRef(3,:)));
% longitud=[len1 len2 len3];
% [valor pos]=max(longitud);
% if(pos==1)
%     pos1=1;
%     pos2=3;
%     pos3=2;
%     pos4=3;
% end
% if(pos==2)
%     pos1=1;
%     pos2=2;
%     pos3=2;
%     pos4=3;
% end
% if(pos==3)
%     pos1=1;
%     pos2=2;
%     pos3=1;
%     pos4=3;
% end
% plot([centrosRef(pos1,1);centrosRef(pos2,1)],[centrosRef(pos1,2);centrosRef(pos2,2)],'LineWidth',2,'Color','green');
% plot([centrosRef(pos3,1);centrosRef(pos4,1)],[centrosRef(pos3,2);centrosRef(pos4,2)],'LineWidth',2,'Color','green');
% plot(centers(:,1),centers(:,2),'gx')
% catch
% end
