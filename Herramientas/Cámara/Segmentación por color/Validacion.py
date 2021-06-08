#Bibliotecas requeridas
import cv2
import re
import math
import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import cm
from matplotlib import colors
from matplotlib.colors import hsv_to_rgb
import os
#-------------------------Definición de funciones---------------------------------
#La función switch retorna el número de robot de una combinación de círculo
#grande y círculo pequeño
def switch(numColor1, numColor2):
    if numColor1==1:
        if numColor2==2:
            robot=11 #cyan magenta
        elif numColor2==3:
            robot=10 #cyan naranja
        elif numColor2==4:
            robot=4  #cyan amarillo
        else:
            #caso no identificado con combinaciones actuales
            robot=999
    elif numColor1==2:
        if numColor2==1:
            robot=12 #magenta cyan
        elif numColor2==3:
            robot=8  #magenta naranja
        elif numColor2==4:
            robot=6  #magenta amarillo
        else:
            #caso no identificado con combinaciones actuales
            robot=999
    elif numColor1==3:
        if numColor2==1:
            robot=9  #naranja cyan
        elif numColor2==2:
            robot=7  #naranja magenta
        elif numColor2==4:
            robot=2  #naranja amarillo
        else:
            #caso no identificado con combinaciones actuales
            robot=999
    elif numColor1==4:
        if numColor2==1:
            robot=3  #amarillo cyan
        elif numColor2==3:
            robot=1  #amarillo naranja
        elif numColor2==2:
            robot=5  #amarillo magenta
        else:
            #caso no identificado con combinaciones actuales
            robot=999
    else:
        #caso no identificado
        robot=999
    return robot
    
def get_contour_areas(cont):
    #Crea una lista donde se guardarán las áreas de cada contorno
    all_areas= []
    #Recorre la lista de contornos
    for cnt in cont:
        #calcula el área de un contorno
        area= cv2.contourArea(cnt)
        #agrega el área calculada al arreglo de áreas
        all_areas.append(area)
    return all_areas
def depurar_contornos(cnts):
    #en esta lista se almacenarán los contornos pertenecientes a los círculos
    #la "d" significa depurados.
    cntsd= []
    #recorro todos los contornos encontrados
    for cnt in cnts:
        area= cv2.contourArea(cnt)
        """
        if area>=500:
            print("Area Encontrada", area)"""
        #Se encontró que el área mínima de los círculos pequeños es de 5000 unidades IMAGEN CON ZOOM
        #Se encontró que el área  de los círculos grandes es de 7000-12000 unidades aprox IMAGEN CON ZOOM
        #OJO hubo cambio del kernel de la dilatación, de 3,3 a 7,7
        #Se encontró que el área mínima de los círculos pequeños es mayor a 900 unidades aprox IMAGEN SIN ZOOM
        #Se encontró que el área  de los círculos grandes es mayor a 1200 unidades aprox IMAGEN SIN ZOOM
        #cuadradas, daremos un margen de error.
        #if area>=4000: #para imagen con zoom
        if area>=850:
            cntsd.append(cnt)
    return cntsd
def centros_contornos(cnts, areas):
    centros= []
    #contador
    cont=0
    for s in cnts:
        #calcula los momentos
        M = cv2.moments(s)
        #Calcula el centroide
        #https://docs.opencv.org/3.4/dd/d49/tutorial_py_contour_features.html
        cx = int(M['m10']/M['m00'])
        cy = int(M['m01']/M['m00'])
        #verifica si el contorno es de un círculo pequeño o grande:
        #if areas[cont]<=6000: #CON ZOOM
        if areas[cont]<=1200: #SIN ZOOM
            #osea círculo pequeño: se le dará un indicador 0
            circ=0
        else:
            #al círculo grande: se le dará un indicador 1
            circ=1
        centros.append((cx, cy, circ))
        cont+=1
    return centros
def calcular_angulos(centros1,centros2,robot):
    return True

def desplazamiento(centerIm1,centerIm2, centros, robot, imagen):
    #centerIm1 e Im2 son las posiciones den las listas de centros
    desplazamientos=[]
    #extracción de las coordenadas de los centros en ambas imágenes (anterior y posterior)
    #print("Centro Imagen 1: ", centerIm1)
    #center da la posición del centro en la imagen. El 0 indica coordena x, y el 1, y.
    x1=centros[imagen][centerIm1][0]
    x2=centros[imagen+1][centerIm2][0]
    y1=centros[imagen][centerIm1][1]
    y2=centros[imagen+1][centerIm2][1]
    #print("x1,x2,y1,y2", x1, x2, y1,y2)
    distance = math.sqrt( ((x1-x2)**2)+((y1-y2)**2) )
    #pasa distancias a mm CON ZOOM
    #distancemm=mmPixelZ*distance
    #pasa distancias a mm SIN ZOOM
    distancemm=mmPixel*distance
    #retorna el número de imagen, el robot y cuánto se movió
    out=(imagen, robot, distancemm)
    desplazamientos.append(out)
    return desplazamientos

def calcular_desplazamiento(centrosC,centrosM,centrosA,centrosN,numImagenes):
    #crea la lista donde se almacenarán los desplazamientos entre 2 imágenes del
    #mismo identificador.
    #n contiene el número de imágenes
    desplazamientos=[]
    #lista de números de robots
    Robots=[]
    
    #analiza todas las imágenes en pares: momento anterior y momento posterior, hasta n-1 imágenes
    #en el caso de solicitud de Cyan
    #buscaremos Cyan con el resto de colores:
    
    #cada peers contendrá el número de imagen, los colores pareja,
    #el centro x,y de cada contorno que es pareja. Peers devuelve
    #desde 0 parejas, hasta el número de combinaciones de un color de
    #círculo grande con el resto de colores.
    CMpeers=buscar_peer(1, 2, centrosC,centrosM)
    #calcula desplazamientos solo si encontró robots
    if len(CMpeers)!=0:
        #print("CMpeers imagen 1",CMpeers[0])
        #se asume el robot NO desaparece de la escena
        #k es el número de imagen
        #for k in range (numImagenes-1):
        for k in range (len(CMpeers)-1):
            #pareja es el número de pareja en la k-ésima imagen
            for pareja in range(len(CMpeers[k])):
                #la última posición que es 2 indica que extrae el centro del círculo grande
                centerIm1=CMpeers[k][pareja][2]
                #extrae el centro grande de la pareja pero en la imagen siguiente
                centerIm2=CMpeers[k+1][pareja][2]
                #la posición 1 indica el número de robot al que corresponde la pareja
                robot= CMpeers[k][pareja][1]
                """#la posición 0 indica el número de imagen
                imagen= CMpeers[k][pareja][0]"""
                #movimiento devuelve el #imagen, el robot y cuánto se desplazó
                movimiento=desplazamiento(centerIm1,centerIm2, centrosC, robot, k)
                desplazamientos.append(movimiento)
    """
    CNpeers=buscar_peer(1, 3, centrosC,centrosN)
    CApeers=buscar_peer(1, 4, centrosC,centrosA)
    """
    #buscaremos Magenta con Amarillo:
    MApeers=buscar_peer(2, 4, centrosM,centrosA)
    #buscaremos Amarillo con Naranja:
    ANpeers=buscar_peer(4, 3, centrosA,centrosN)
    #----------------------------Pruebas Rebeca-----------------------------
    #buscaremos Cyan con Amarillo:
    CApeers=buscar_peer(1, 4, centrosC,centrosA)
    #buscaremos Amarillo con Magenta:
    AMpeers=buscar_peer(4, 2, centrosA,centrosM)
    
    #Para Cyan con Amarillo
    if len(CApeers)!=0:
        #print("CMpeers imagen 1",CMpeers[0])
        #se asume el robot NO desaparece de la escena
        #k es el número de imagen
        for k in range (numImagenes-1):
            #pareja es el número de pareja en la k-ésima imagen
            for pareja in range(len(CApeers[k])):
                #la última posición que es 2 indica que extrae el centro del círculo grande
                centerIm1=CApeers[k][pareja][2]
                #extrae el centro grande de la pareja pero en la imagen siguiente
                centerIm2=CApeers[k+1][pareja][2]
                #la posición 1 indica el número de robot al que corresponde la pareja
                robot= CApeers[k][pareja][1]
                """#la posición 0 indica el número de imagen
                imagen= CMpeers[k][pareja][0]"""
                #movimiento devuelve el #imagen, el robot y cuánto se desplazó
                movimiento=desplazamiento(centerIm1,centerIm2, centrosC, robot, k)
                desplazamientos.append(movimiento)

    return desplazamientos

def escribir_txt(nombretxt,desplazamientos):
    with open(nombretxt, 'w') as output:
        output.write("Desplazamientos: #imagen; robot; desplazamiento; ángulo"+ '\n')
        for d in desplazamientos:
            print("desplazamiento d: ", d)
            imagen=d[0][0]
            robot=d[0][1]
            desplazamiento=d[0][2]
            output.write(str(imagen) + ";"+str(robot)+ ";"+ str(desplazamiento)+";"+ "ángulo"+ '\n')
        print("Archivo creado")
def buscar_peer(numColor1, numColor2, centrosC1,centrosC2):
    #la lista de peers contendrá los colores que son pareja (su identificador), y la posición
    #de cada círculo en la lista de centros.
    peers=[]
    for imagen in range(len(centrosC1)):
        #almacenará todos los peers de una imagen para los 2 colores dados
        peersXimagen=[]
        for centro in range(len(centrosC1[imagen])):
            for centro2 in range(len(centrosC2[imagen])):
                #extracción de las coordenadas de los centros de ambos colores
                x1=centrosC1[imagen][centro][0]
                x2=centrosC2[imagen][centro2][0]
                y1=centrosC1[imagen][centro][1]
                y2=centrosC2[imagen][centro2][1]
                #print("x1,x2,y1,y2", x1, x2, y1,y2)
                distance = math.sqrt( ((x1-x2)**2)+((y1-y2)**2) )
                """if centro2==0:
                    print("distancia calculada de peers", distance)"""
                #CON ZOOM
                #if (distance<=70) and (centrosC2[imagen][centro2][2]==0):
                #140 píxeles mide de diámetro aproximadamente un círculo grande
                #se aproxima la mitad de esta distancia como una distancia prudencial
                #para encontrar un contorno de círculo pequeño. Además, la distancia aproximada entre centros
                #de círculos grande y pequeño es de 60 (comprobado por paint y este cálculo). Se agregará un pequeño margen de 5 píxeles
                #SIN ZOOM
                #55 píxeles mide de diámetro aproximadamente un círculo grande
                #se aproxima la mitad de esta distancia como una distancia prudencial
                #para encontrar un contorno de círculo pequeño. Además, la distancia aproximada entre centros
                #de círculos grande y pequeño es de 30 (comprobado por paint y este cálculo). Se agregará un pequeño margen de 5 píxeles
                #Esta función solo agrega al peer si la distancia se cumple y el primero es el más grande, así
                #se evita la duplicidad de peers
                if (distance<=35) and (centrosC2[imagen][centro2][2]==0):
                    #guarda el número de imagen en cuestión, los colores que hicieron match,
                    #las posiciones dentro de los centros de cada color que hicieron match.
                    #identificar #robot:
                    robot=switch(numColor1, numColor2)
                    peer=(imagen, robot, centro, centro2)
                    """if centro2==0:
                        print("peer encontrado", peer)"""
                    #agrega cada peer encontrado a la lista de peers
                    peersXimagen.append(peer)
        peers.append(peersXimagen)
    #al final peers tendrá un número de elementos igual al del número de imágenes
    #cada elemento es una lista de parejas encontradas para los 2 colores solicitados
    return peers

        
def identificar_robots(Acyan, Amagenta, Aamarillo, Anaranja, centros_cCyan, centros_cMagenta, centros_cNaranja, centros_cAmarillo):
    #recibe las listas de áreas de los contornos depurados y ordenados.
    #cada lista en la posición [i] tiene la lista de áreas de los contornos de una imagen
    #cada lista en la posición [i][j] tiene una área de un contorno, entre mayor j mayor tamaño de contorno.
    #Se asume que la cantidad de robots no cambia en las imágenes, por eso índice [0] -EN REVISIÓN-
    #primera dimensión es la cantidad de imágenes, y la segunda es la cantidad de contornos de ese color
    #140 píxeles mide de diámetro aproximadamente un círculo grande: Imagen con ZOOM
                  
    return True
#calculo de angulo entre rectas
#https://byjus.com/maths/angle-between-two-lines/

def obtenerFrames(nombreVideoIn, nombreCarpetaOut):
    #Se extraen los cuadros del video y se guardan
    #El vídeo está en 34 fps
    
    cap= cv2.VideoCapture(nombreVideoIn)
    n=0
    contFrames=0
    contImagenes=0
    #cada cuántos frames guardo una imagen
    espaciado=90*2  #aprox cada 6 segundos tomando en cuenta 34fps  #REBECA
    while(cap.isOpened()):
        ret, frame = cap.read()
        if ret == False:
            break
        #este
        if contFrames==espaciado:
            cv2.imwrite(str(nombreCarpetaOut)+'/frame'+str(contImagenes)+'.jpg',frame)
            contImagenes+=1
            contFrames=0
        n+=1
        contFrames+=1
    cap.release()
    cv2.destroyAllWindows()
    

    #Se cargan los cuadros del video en una colección
    #primero se guardan los nombres de las imagenes
    col_images = []
    col_frames = os.listdir(str(nombreCarpetaOut))

    # se ordenan por nombre, en este caso se guardaron con numeración consecutiva
    col_frames.sort(key=lambda f: int(re.sub('\D', '', f)))
    #returna el número de frames de la imagen
    #con este número y la duración del vídeo se obtienen los fps
    return n
        

#--------------------- Lectura de imágenes-----------------------------------------
#Nombre vídeo en la carpeta de Segmentación por color   #REBECA
nombreVideoIn='VideoP3P2.mp4'
nombreCarpetaOut='ImagenesRbk/'

#---------------------------PARA PROBAR OTRO VÍDEO
#Obtener Frames de un vídeo: Solo se ejecuta si es la primera vez con este vídeo
#numFrames=obtenerFrames(nombreVideoIn, nombreCarpetaOut)
#-------------------------------------------------


#Dirección de la carpeta con las imágenes a utilizar

#Dirección de imágenes prueba Bryan
#dir = 'Imagenes/'
#Dirección de imágenes de prueba Rebeca
dir = nombreCarpetaOut
#Guarda los nombres de las imágenes que estaban en la carpeta en la lista imgList
imgList = os.listdir(dir)
#Inicializa la lista que contendrá a las imágenes que se utilizarán
imagenes = []
imagenesHSV = []
#Ordena las imágenes por número de menor a mayor
imgList.sort (key = lambda x: int (x.replace ("frame", ""). split ('.') [0])) 
#Inicializa contadores
n = 0
cont = 0
ind = 0
#resolución espacial para imagen con zoom [mm/pixel]
mmPixelZ=40/54
#resolución espacial para imagen sin zoom [mm/pixel]
mmPixel=40/36
for i in imgList:
    #va contando los frames, prevista para frames
    #cont+=1
    #Lee cada imagen, donde i es el nombre de cada una
    img = cv2.imread(nombreCarpetaOut+i)
    #Prevista para cada cuantos frames se quiere trabajar
    """if cont == 4:
        imagenes.append(img)     
        cont = 0
        n+=1"""
    #Convirtiendo a RGB
    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    hsv = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)
    #Guarda las imágenes en RGB en una lista
    imagenes.append(img)
    #Guarda las imágenes en HSV en una lista
    imagenesHSV.append(hsv)
    #va contando las imágenes que hay
    n+=1

#--------------------- Definición de rangos de color HSV---------------------------
#Rangos cyan:
light_cyan = (95, 175, 185)
dark_cyan = (105, 255, 250)
#Rangos magenta:
light_magenta = (160, 100, 0)
dark_magenta = (170, 255, 255)
#Rangos verde:
light_verde = (45, 20, 0)
dark_verde = (80, 100, 255)
#Rangos rojo:
light_rojo = (168, 50, 0)
dark_rojo = (180, 255, 255)
#Rangos azul: 
light_azul = (100, 125, 10)
dark_azul = (120, 255, 255)
#Rangos naranja: 
light_nar = (12, 100, 200)
dark_nar = (16, 255, 255)
#Rangos amarillo: 
#light_ama = (15, 30, 0)
light_ama = (15, 30, 175)
dark_ama = (35, 255, 255)
#Crea un respaldo de todas las imágenes para ser modificadas más adelante
RespaldoImagenes=list(imagenes)

#------------Procesado: aplicar máscara, dilatar, contornos, ubicación -----------------
#Aquí se almacenarán los contornos de cada una de las imágenes
lista_cCyan = []
lista_cMagenta = []
lista_cVerde = []
lista_cRojo = []
lista_cAzul = []
lista_cNaranja = []
lista_cAmarillo = []
#Aquí se almacenarán los centros de los contornos
centros_cCyan = []
centros_cMagenta = []
centros_cNaranja = []
centros_cAmarillo = []
centros_cVerde = []
centros_cRojo = []
centros_cAzul = []

for j in range(len(imagenes)):
    #Máscara Cyan
    #Pone 1 en el píxel que está dentro del rango, sino pone 0
    maskcyan = cv2.inRange(imagenesHSV[j], light_cyan, dark_cyan)
    #Máscara Magenta
    maskmagenta = cv2.inRange(imagenesHSV[j], light_magenta, dark_magenta)
    #Máscara Verde
    maskverde = cv2.inRange(imagenesHSV[j], light_verde, dark_verde)
    #Máscara Amarilla
    maskama = cv2.inRange(imagenesHSV[j], light_ama, dark_ama)
    #Máscara Naranja
    masknar = cv2.inRange(imagenesHSV[j], light_nar, dark_nar)
    #Máscara Azul
    maskazul = cv2.inRange(imagenesHSV[j], light_azul, dark_azul)
    #Máscara Rojo
    maskrojo = cv2.inRange(imagenesHSV[j], light_rojo, dark_rojo)
    #Máscara de todos los colores de identificadores:
    maskAll=maskrojo+maskazul+maskama+maskcyan+maskverde+maskmagenta+masknar

    #Se utiliza una transformación morfológica de dilatación de la imagen para hacer los
    #contornos más uniformes
    #Se crea el kernel, entre mayor sea más "grosera" es la dilatación
    kernel = np.ones((7,7),np.uint8)
    cyand = cv2.dilate(maskcyan,kernel,iterations=1)
    magentad = cv2.dilate(maskmagenta,kernel,iterations=1)
    verded = cv2.dilate(maskverde,kernel,iterations=1)
    amarillod = cv2.dilate(maskama,kernel,iterations=1)
    naranjad = cv2.dilate(masknar,kernel,iterations=1)
    azuld = cv2.dilate(maskazul,kernel,iterations=1)
    rojod = cv2.dilate(maskrojo,kernel,iterations=1)
    alld = cyand+magentad+verded+amarillod+naranjad+azuld+rojod
    
    #Aplica las máscaras a las imágenes por si se quisieran visualizar
    #"""
    resultcyan = cv2.bitwise_and(imagenes[j], imagenes[j], mask=cyand)
    resultmagenta = cv2.bitwise_and(imagenes[j], imagenes[j], mask=magentad)
    resultverde = cv2.bitwise_and(imagenes[j], imagenes[j], mask=verded)
    resultama = cv2.bitwise_and(imagenes[j], imagenes[j], mask=amarillod)
    resultnar = cv2.bitwise_and(imagenes[j], imagenes[j], mask=naranjad)
    resultazul = cv2.bitwise_and(imagenes[j], imagenes[j], mask=azuld)
    resultrojo = cv2.bitwise_and(imagenes[j], imagenes[j], mask=rojod)
    #Aplica la máscara combinada
    resultAll = cv2.bitwise_and(imagenes[j], imagenes[j], mask=maskAll)
    
    #Muestra los resultados de filtrado por color para una imagen
    
    if ind==0:
        # se crea la figura
        fig = plt.figure(figsize=(8, 8))
        # definición de filas y columnas
        rows = 4
        columns = 2
        # Agrega un subplot en la primera posición
        fig.add_subplot(rows, columns, 1)
        # mostrando la imagen
        plt.imshow(resultcyan)
        plt.axis('off')
        plt.title("Cyan")
        # segunda posición
        fig.add_subplot(rows, columns, 2)
        plt.imshow(resultmagenta)
        plt.axis('off')
        plt.title("Magenta")
        # tercera posición
        fig.add_subplot(rows, columns, 3)
        plt.imshow(resultverde)
        plt.axis('off')
        plt.title("Verde")
        # cuarta posición
        fig.add_subplot(rows, columns, 4)
        plt.imshow(resultama)
        plt.axis('off')
        plt.title("Amarillo")
        # quinta posición
        fig.add_subplot(rows, columns, 5)
        plt.imshow(resultnar)
        plt.axis('off')
        plt.title("Naranja")
        # sexta posición
        fig.add_subplot(rows, columns, 6)
        plt.imshow(resultazul)
        plt.axis('off')
        plt.title("Azul")
        # séptima posición
        fig.add_subplot(rows, columns, 7)
        plt.imshow(resultrojo)
        plt.axis('off')
        plt.title("Rojo")
        # octava posición
        fig.add_subplot(rows, columns, 8)
        plt.imshow(resultAll)
        plt.axis('off')
        plt.title("Todos")
        plt.show()
        #"""
    ind+=1
    
    #se inicializa la variable de tamaño de contorno
    sizeC=0
    #Busca los contornos en cada una de las imágenes filtradas
    #Se aproximan por medio de una compresión horizontal, vertical y diagonal
    #https://docs.opencv.org/master/d3/dc0/group__imgproc__shape.html#gadf1ad6a0b82947fa1fe3c3d497f260e0
    cntsCyan, jerCyan = cv2.findContours(cyand.copy(),cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE) 
    cntsMagenta, jerMagenta = cv2.findContours(magentad.copy(),cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    cntsAmarillo, jerAmarillo = cv2.findContours(amarillod.copy(),cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE) 
    cntsNaranja, jerNaranja = cv2.findContours(naranjad.copy(),cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    #Deja solo los contornos grandes, correspondientes a los círculos pequeños y grandes
    cntsCyans=depurar_contornos(cntsCyan)
    cntsMagentas=depurar_contornos(cntsMagenta)
    cntsAmarillos=depurar_contornos(cntsAmarillo)
    cntsNaranjas=depurar_contornos(cntsNaranja)
    #Se ordenan los contornos por área, la "s" al final de la nueva variable es de "sorted"
    #sorted ordena de menor a mayor por área los contornos.
    cntsCyans = sorted(cntsCyans, key=lambda x: cv2.contourArea(x))
    cntsMagentas = sorted(cntsMagentas, key=lambda x: cv2.contourArea(x))
    cntsAmarillos = sorted(cntsAmarillos, key=lambda x: cv2.contourArea(x))
    cntsNaranjas = sorted(cntsNaranjas, key=lambda x: cv2.contourArea(x))
    #Los colores Azul, Rojo y Verde (al menos no el tono deseado), no están en las imágenes de prueba actuales
    """
    cntsAzul, jerAzul = cv2.findContours(azuld.copy(),cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE) 
    cntsAzuls = sorted(cntsAzul, key=lambda x: cv2.contourArea(x))
    cntsRojo, jerRojo = cv2.findContours(rojod.copy(),cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE) 
    cntsRojos = sorted(cntsRojo, key=lambda x: cv2.contourArea(x))
    cntsVerde, jerVerde = cv2.findContours(verded.copy(),cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE) 
    cntsVerdes = sorted(cntsVerde, key=lambda x: cv2.contourArea(x))
    """
    #Se agregan los contornos encontrados por cada color a la lista de contornos correspondiente
    lista_cCyan.append(cntsCyans)
    lista_cMagenta.append(cntsMagentas)
    lista_cNaranja.append(cntsNaranjas)
    lista_cAmarillo.append(cntsAmarillos)
    """
    lista_cAzul.append(cntsCyans)
    lista_cVerde.append(cntsCyans)
    lista_cRojo.append(cntsCyans)
    """
    #Calcula el área de los contornos encontrados
    areasCyan=get_contour_areas(cntsCyans)
    #Para vídeo con zoom
    #Se observa que las áreas pequeñas (círculos pequeños), son de al menos 5000 unidades cuadradas,
    #mientras que las áreas de círculos grandes están en el orden de 7000-12000 unidades cuadradas
    #Para vídeo SIN zoom (SOLICITUD PRIORITARIA DE PARTE PROFESOR JUAN CARLOS 07/06/21)
    #Se observa que las áreas pequeñas (círculos pequeños), son de al menos 5000 unidades cuadradas,
    #mientras que las áreas de círculos grandes están en el orden de 7000-12000 unidades cuadradas
    #
    #Por este motivo, se proceden a eliminar los contornos que no cumplan estos criterios.
    areasMagenta=get_contour_areas(cntsMagentas)
    areasAmarillo=get_contour_areas(cntsAmarillos)
    areasNaranja=get_contour_areas(cntsNaranjas)

    print("Áreas encontradas Amarillo", areasAmarillo)
    print("Áreas encontradas Cyan", areasCyan)
    """
    print("Áreas Cyan depuradas y ordenadas", areasMagenta)
    
    print("Áreas encontradas Naranja", areasNaranja)
    print("Áreas encontradas Magenta", areasMagenta)
    """

    #Dibuja los contornos encontrados correspondientes
    #El -1 significa que dibuje todos los contornos. EL 0,0,255 es el color, y 3 es el grosor
    #lineas azules para contornos cyan
    contornosCyan = cv2.drawContours(RespaldoImagenes[j], cntsCyans, -1, (0,0,255), 5)
    #lineas negras para contornos amarillos
    contornosAmarillo = cv2.drawContours(RespaldoImagenes[j], cntsAmarillos, -1, (0,0,0), 5)
    #lineas rojas para contornos naranjas
    contornosNaranja = cv2.drawContours(RespaldoImagenes[j], cntsNaranjas, -1, (255,0,0), 5)
    #lineas moradas para contornos magenta
    contornosMagenta = cv2.drawContours(RespaldoImagenes[j], cntsMagentas, -1, (155,0,255), 5)

    #Muestra los resultados parciales de la primera imagen2
    if j==0:
        """
        #Para mostrar subplots
        # se crea la figura
        fig = plt.figure(figsize=(8, 8))
        # definición de filas y columnas
        rows = 2
        columns = 2
        # Agrega un subplot en la primera posición
        fig.add_subplot(rows, columns, 1)
        # mostrando la imagen
        plt.imshow(contornosCyan)
        plt.axis('off')
        plt.title("Contornos Cyan")
        # segunda posición
        fig.add_subplot(rows, columns, 2)
        plt.imshow(contornosMagenta)
        plt.axis('off')
        plt.title("Contornos Magenta")
        # tercera posición
        fig.add_subplot(rows, columns, 3)
        plt.imshow(contornosNaranja)
        plt.axis('off')
        plt.title("Contornos Naranja")
        # cuarta posición
        fig.add_subplot(rows, columns, 4)
        plt.imshow(contornosMagenta)
        plt.axis('off')
        plt.title("Contornos Amarillo")
        plt.show()
        """
        #Mostrando los contornos dibujados
        """
        fig = plt.figure(figsize=(8, 8))
        rows = 1
        columns = 1
        fig.add_subplot(rows, columns, 1)
        plt.imshow(contornosMagenta)
        plt.axis('off')
        plt.title("Contornos encontrados")
        plt.show()
        """
    #Cálculo de los centroides de los contornos
    centrosCyan=centros_contornos(cntsCyans, areasCyan)
    centrosMagenta=centros_contornos(cntsMagentas, areasMagenta)
    centrosAmarillo=centros_contornos(cntsAmarillos, areasAmarillo)
    centrosNaranja=centros_contornos(cntsNaranjas, areasNaranja)
    #Agrego los centros de cada color a su respectiva lista que contiene a los
    #centros de todas las imágenes
    #Fueron verificados en paint: validados!
    centros_cCyan.append(centrosCyan)
    centros_cMagenta.append(centrosMagenta)
    centros_cAmarillo.append(centrosAmarillo)
    centros_cNaranja.append(centrosNaranja)
"""
print("Centroides Cyan",centros_cCyan)
print("Centroides Magenta",centros_cMagenta)
print("Centroides Amarillo",centros_cAmarillo)
print("Centroides Naranja",centros_cNaranja)
"""
"""
#Cálculo de desplazamientos por identificador
dCyan=calcular_desplazamiento(centros_cCyan, centros_cMagenta, centros_cAmarillo, centros_cNaranja,n, 1 )
dMagenta=calcular_desplazamiento(centros_cCyan, centros_cMagenta, centros_cAmarillo, centros_cNaranja,n, 2  )
dAmarillo=calcular_desplazamiento(centros_cCyan, centros_cMagenta, centros_cAmarillo, centros_cNaranja,n, 4  )
dNaranja=calcular_desplazamiento(centros_cCyan, centros_cMagenta, centros_cAmarillo, centros_cNaranja,n, 3 )
#print("Desplazamientos Cyan", desplazamientosCyan)
"""

#Cálculo de desplazamientos: de momento solo Cyan se implementó en la función
desplazamientosX=calcular_desplazamiento(centros_cCyan, centros_cMagenta, centros_cAmarillo, centros_cNaranja,n)

#Buscar parejas de cada círculo
#Cada color se identifica con un número: cyan (1), magenta (2), naranja (3), amarillo (4)
#cada combinación se identifica con un número de 0-12 de momento
#EN USO significa que es una combinación que está en las fotos
#el primer color es el círculo grande, y el segundo el pequeño
#EN USO amarillo-naranja [1], naranja-amarillo [2]
#amarillo-magenta [5],  magenta-amarillo [6]
#EN USOcyan-magenta [11], magenta-cyan [12]

#buscaremos Cyan con Magenta:
CMpeers=buscar_peer(1, 2, centros_cCyan,centros_cMagenta)
#mostraremos los peers encontrados para la primera imagen:
print("Peers imagen 1 Cyan-Magenta", CMpeers[0])

#buscaremos Magenta con Amarillo:
MApeers=buscar_peer(2, 4, centros_cMagenta,centros_cAmarillo)
#mostraremos los peers encontrados para la primera imagen:
print("Peers imagen 1 Magenta-Amarillo", MApeers[0])
#tiene sentido ya que encontró el contorno más grande magenta, con
#el contorno más pequeño amarillo.

#buscaremos Amarillo con Naranja:
ANpeers=buscar_peer(4, 3, centros_cAmarillo,centros_cNaranja)
#mostraremos los peers encontrados para la primera imagen:
print("Peers imagen 1 Amarillo-Naranja", ANpeers[0])

#Crea archivo txt con los desplazamientos
#Nombre del txt:
nombretxt="Desplazamientos.txt"
escribir_txt(nombretxt,desplazamientosX)






    
