#Bibliotecas requeridas
import cv2
import math
import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import cm
from matplotlib import colors
from matplotlib.colors import hsv_to_rgb
import os
#-------------------------Definición de funciones---------------------------------
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
        #Se encontró que el área mínima de los círculos pequeños es de 5000 unidades
        #cuadradas, daremos un margen de error.
        if area>=4000:
            cntsd.append(cnt)
    return cntsd
def centros_contornos(cnts):
    centros= []
    for s in cnts:
        #calcula los momentos
        M = cv2.moments(s)
        #Calcula el centroide
        #https://docs.opencv.org/3.4/dd/d49/tutorial_py_contour_features.html
        cx = int(M['m10']/M['m00'])
        cy = int(M['m01']/M['m00'])
        centros.append((cx, cy))
    return centros

def calcular_desplazamiento(centros,numImagenes):
    #crea la lista donde se almacenarán los desplazamientos entre 2 imágenes del
    #mismo identificador.
    #n contiene el número de imágenes
    desplazamientos=[]
    for k in range (n-1):
        for i in range(len(centros[k])):
            #extracción de las coordenadas de los centros en ambas imágenes
            x1=centros[k][i][0]
            x2=centros[k+1][i][0]
            y1=centros[k][i][1]
            y2=centros[k+1][i][1]
            #print("x1,x2,y1,y2", x1, x2, y1,y2)
            distance = math.sqrt( ((x1-x2)**2)+((y1-y2)**2) )
            desplazamientos.append(distance)
    return desplazamientos

def escribir_txt(nombretxt,dcyan,dmagenta,damarillo, dnaranja):
    with open(nombretxt, 'w') as output:
        output.write("Desplazamientos Cyan"+ '\n')
        for d in dcyan:
            output.write(str(d) + '\n')
        output.write("Desplazamientos Magenta"+ '\n')
        for e in dmagenta:
            output.write(str(e) + '\n')
        output.write("Desplazamientos Amarillo"+ '\n')
        for f in damarillo:
            output.write(str(f) + '\n')
        output.write("Desplazamientos Naranja"+ '\n')
        for g in dmagenta:
            output.write(str(g) + '\n')
        print("Archivo creado")
        

#--------------------- Lectura de imágenes-----------------------------------------
#Dirección de la carpeta con las imágenes a utilizar
dir = 'Imagenes/'
#Guarda los nombres de las imágenes que estaban en la carpeta en la lista imgList
imgList = os.listdir(dir)
#Inicializa la lista que contendrá a las imágenes que se utilizarán
imagenes = []
imagenesHSV = []
#Ordena las imágenes por número de menor a mayor
imgList.sort (key = lambda x: int (x.replace ("AmNaCyMaVe", ""). split ('.') [0])) 
#Inicializa contadores
n = 0
cont = 0
ind = 0
for i in imgList:
    #va contando los frames, prevista para frames
    #cont+=1
    #Lee cada imagen, donde i es el nombre de cada una
    img = cv2.imread('Imagenes/'+i)
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
light_ama = (15, 30, 0)
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
centros_cVerde = []
centros_cRojo = []
centros_cAzul = []
centros_cNaranja = []
centros_cAmarillo = []
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
    kernel = np.ones((3,3),np.uint8)
    cyand = cv2.dilate(maskcyan,kernel,iterations=1)
    magentad = cv2.dilate(maskmagenta,kernel,iterations=1)
    verded = cv2.dilate(maskverde,kernel,iterations=1)
    amarillod = cv2.dilate(maskama,kernel,iterations=1)
    naranjad = cv2.dilate(masknar,kernel,iterations=1)
    azuld = cv2.dilate(maskazul,kernel,iterations=1)
    rojod = cv2.dilate(maskrojo,kernel,iterations=1)
    alld = cyand+magentad+verded+amarillod+naranjad+azuld+rojod
    #Aplica las máscaras a las imágenes por si se quisieran visualizar
    """
    resultcyan = cv2.bitwise_and(imagenes[j], imagenes[j], mask=cyand)
    resultmagenta = cv2.bitwise_and(imagenes[j], imagenes[j], mask=magentad)
    resultverde = cv2.bitwise_and(imagenes[j], imagenes[j], mask=verded)
    resultama = cv2.bitwise_and(imagenes[j], imagenes[j], mask=amarillod)
    resultnar = cv2.bitwise_and(imagenes[j], imagenes[j], mask=naranjad)
    resultazul = cv2.bitwise_and(imagenes[j], imagenes[j], mask=azuld)
    resultrojo = cv2.bitwise_and(imagenes[j], imagenes[j], mask=rojod)
    #Aplica la máscara combinada
    resultAll = cv2.bitwise_and(imagenes[j], imagenes[j], mask=maskAll)
    """
    #Muestra los resultados de filtrado por color para una imagen
    """
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
        """
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
    lista_cMagenta.append(cntsCyans)
    lista_cNaranja.append(cntsCyans)
    lista_cAmarillo.append(cntsCyans)
    """
    lista_cAzul.append(cntsCyans)
    lista_cVerde.append(cntsCyans)
    lista_cRojo.append(cntsCyans)
    """
    #Calcula el área de los contornos encontrados
    areasCyan=get_contour_areas(cntsCyans)
    #Se observa que las áreas pequeñas (círculos pequeños), son de al menos 5000 unidades cuadradas,
    #mientras que las áreas de círculos grandes están en el orden de 7000-12000 unidades cuadradas
    #Por este motivo, se proceden a eliminar los contornos que no cumplan estos criterios.
    """
    print("Áreas encontradas Cyan", areasCyan)
    areasAmarillo=get_contour_areas(cntsAmarillos)
    print("Áreas encontradas Amarillo", areasAmarillo)
    areasNaranja=get_contour_areas(cntsNaranjas)
    print("Áreas encontradas Naranja", areasNaranja)
    areasMagenta=get_contour_areas(cntsMagentas)
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
    centrosCyan=centros_contornos(cntsCyans)
    centrosMagenta=centros_contornos(cntsMagentas)
    centrosAmarillo=centros_contornos(cntsAmarillos)
    centrosNaranja=centros_contornos(cntsNaranjas)
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
#Cálculo de desplazamientos por identificador
dCyan=calcular_desplazamiento(centros_cCyan,n)
dMagenta=calcular_desplazamiento(centros_cMagenta,n)
dAmarillo=calcular_desplazamiento(centros_cAmarillo,n)
dNaranja=calcular_desplazamiento(centros_cNaranja,n)
#print("Desplazamientos Cyan", desplazamientosCyan)

#Crea archivo txt con los desplazamientos
#Nombre del txt:
nombretxt="Desplazamientos.txt"
escribir_txt(nombretxt,dCyan,dMagenta,dAmarillo, dNaranja)






    
