%En el presente script se realiza el proceso de calibración de la cámara,
%con el fin de eliminar distorsiones y obtener los parámetros que
%relacionan las imágenes capturadas con las dimensiones reales.
%Esto se requiere para su posterior uso en la medición de distancias
%y ángulos para el análisis de trayectorias de un enjambre de robots del
%proyecto PROE

%-------------------------- Notas relevantes-------------------------------

%El objetivo es realizar la toma de imágenes para calibrar la cámara
%utilizada.

%Es necesario capturar una cantidad de ejemplos útiles para que la aplicación
%determine los parámetros correspondientes. 

%Si se usa DroidCam, debe iniciarse el cliente de windows y conectarse al dispositivo móvil.
%Ambos dispositivos deben estar conectados a la misma red Wi-fi

%Links de descarga:

%Para establecer una conexión
%Cliente Windows: https://www.dev47apps.com/droidcam/windows/
%Droidcam Play Store: https://play.google.com/store/apps/details?id=com.dev47apps.droidcam
%Droidcam App Store: https://apps.apple.com/us/app/droidcam-wireless-webcam/id1510258102

%Se requiere imprimir en escala real el patrón de calibración llamado
%"Visión Patrón". Este patrón debe aparecer en las imágenes capturadas.

%La versión de Matlab debe ser al menos 2015.

%Debe estar instalado el Matlab Support Package for USB Webcams, que se
%obtiene desde la pestaña de Home->Add-Ons.
%También se puede obtener con el enlace:
% https://la.mathworks.com/matlabcentral/fileexchange/45182-matlab-support-package-for-usb-webcams

%-------------------------- Inicio del programa ---------------------------
%A continuación se toman 10 imágenes, con un intervalo de 5 segundos entre
%cada una, para dar tiempo a un acomodo variado de ángulos, inclinaciones
%(bajas) y desplazamientos (cortos), realizados por la persona, dentro del área de
%trabajo y captura de la cámara.

%elimina las sesiones webcam anteriores y variables anteriores
clear all 
%limpia el command window
clc 
camList=webcamlist %Despliega la lista de cámaras disponibles
cam=webcam(2) %Se selecciona la webcam DroidCam (cámara web inalámbrica remota con android)
preview(cam); %Se muestra un recuadro con la imagen en tiempo real
pause('on') %Activa la funcionalidad de pausa, que permite detener x segundos la ejecución
for i = 1:10
        %Se captura una imagen
        img=snapshot(cam); 
        %Se define el nombre de la imagen, según el contador
        filename = ['Image' num2str(i)]; 
        imwrite(img,filename,'png');
        disp('Imagen # '); %Se muestra en pantalla el # de imagen capturada
        disp(i);
        disp(' capturada');
        imshow(filename); %Se muestra la imagen capturada
        if i==10
            disp('Fin de captura de imágenes');
        end
        pause(5); %Espera 5 segundos para volver a tomar otra foto
end
%Si se desea finalizar la ejecución del código de forma anticipada
%presionar Ctrl + C
