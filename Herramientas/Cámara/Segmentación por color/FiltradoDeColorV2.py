#Filtrado de color para identificación de distintos identificadores
#circulares dobles.

#Se pueden encontrar los identificadores en el siguiente enlace:
#https://imgur.com/a/KSHhX7t

#Cada robot móvil cuenta con un identificador doble a color, que
#se compone de 2 círculos inscritos de colores distintos entre sí
#así como del resto del espacio imagen del laboratorio de prueba.

#Mediante este algoritmo se busca identificar y aislar cada robot
#en 2 posiciones distintas, para de esta forma reportar su movimiento
#traslacional, así como su ángulo de giro. Para esto se toman, al menos
#2 fotos, y luego se superpone el resultado del filtrado de color
#específico para ese robot.

#Es decir, una vez se identifica cada robot en cada foto, las fotos filtradas
#para los mismos robots se superponen, lo que permite ver la posición inicial
#y final en la misma imagen, y de esta forma encontrar la posición de los
#círculos.

#Basado en los desarrollos de:
#https://realpython.com/python-opencv-color-spaces/

#Para la identificación de los colores se utilizará el espacio de color
#HSV (Hue, Saturation y Value-Brightness).
#Aunque el espacio de color real es infinito, para trabajar computacionalmente
#se asume discreto.

#Bibliotecas requeridas
import cv2
import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import cm
from matplotlib import colors
from matplotlib.colors import hsv_to_rgb

#Guarda todas las conversiones de espacios de color que tiene OpenCV
flags = [i for i in dir(cv2) if i.startswith('COLOR_')]
#Para ver cuántas hay
#len(flags)
#Cambiando num por un entero positivo se puede saber a cuál corresponde
#esa posición.
#flags[num]

#Cargar la imagen y mostrarla
#imagen con colores impresos
prueba = cv2.imread('RoAzVe.jpg')
#fotografía de cómo se ven los identificadores ya cortados en las condiciones del aula
prueba2= cv2.imread('AmNaCyMaVe1.jpg')
#fotografía de cómo se ven los identificadores ya cortados en las condiciones del aula,
#con los robots movidos
prueba3= cv2.imread('AmNaCyMaVe2.jpg')
#plt.imshow(prueba)
#plt.show()
#Por defecto, OpenCV lee las imágenes en formato BGR, por eso los canales R y B salen
#intercambiados.

#Cambiando a RGB
prueba1 = cv2.cvtColor(prueba, cv2.COLOR_BGR2RGB)
prueba2 = cv2.cvtColor(prueba2, cv2.COLOR_BGR2RGB)
prueba3 = cv2.cvtColor(prueba3, cv2.COLOR_BGR2RGB)
#ver una imagen en rgb
#plt.imshow(magenta1)
#plt.show()

#link de interés:
#https://docs.opencv.org/master/df/d9d/tutorial_py_colorspaces.html
#Convertir RGB a HSV
hsv_prueba1 = cv2.cvtColor(prueba1, cv2.COLOR_RGB2HSV)
hsv_prueba2 = cv2.cvtColor(prueba2, cv2.COLOR_RGB2HSV)
hsv_prueba3 = cv2.cvtColor(prueba3, cv2.COLOR_RGB2HSV)

#Mostrar en 3D el espacio HSV de la imagen
#Normalizar
#Imagen capturada con identificadores impresos
#-----------------------------------------CONVERSIÓN Y VISUALIZACIÓN HSV-------------------------------
pixel_colors = hsv_prueba2.reshape((np.shape(hsv_prueba2)[0]*np.shape(hsv_prueba2)[1], 3))
norm = colors.Normalize(vmin=-1.,vmax=1.)
norm.autoscale(pixel_colors)
pixel_colors = norm(pixel_colors).tolist()

h, s, v = cv2.split(hsv_prueba2)
"""
fig = plt.figure()
axis = fig.add_subplot(1, 1, 1, projection="3d")
axis.scatter(h.flatten(), s.flatten(), v.flatten(), facecolors=pixel_colors, marker=".")
#Hue es un ángulo, de 0 a 179.
axis.set_xlabel("Hue")
#hasta 255
axis.set_ylabel("Saturation")
#value es la intensidad luminosa de 0 a 255
axis.set_zlabel("Value")
plt.show()
"""

#--------------------------------------------RANGOS----------------------------------------------------
#Link de ayuda RGB a HSV aproximado:
#http://www.speakingsame.com/hsv/index.php
#Rangos prueba: funciona detectando todos los identificadores juntos
light_colors = (0, 100, 0)
dark_colors = (180, 255, 255)

#Rangos cyan: probados buenos
light_cyan = (95, 175, 185)
dark_cyan = (105, 255, 250)
#Ver cyan
"""
lc_square = np.full((10, 10, 3), light_cyan, dtype=np.uint8) / 255.0
dc_square = np.full((10, 10, 3), dark_cyan, dtype=np.uint8) / 255.0
plt.subplot(1, 2, 1)
plt.imshow(hsv_to_rgb(dc_square))
plt.subplot(1, 2, 2)
plt.imshow(hsv_to_rgb(lc_square))
plt.show()
#"""

"""
#Rangos magenta: probados buenos
light_magenta = (140, 100, 0)
dark_magenta = (165, 255, 255)
"""
#Rangos magenta: probados buenos
light_magenta = (160, 100, 0)
dark_magenta = (170, 255, 255)

#Ver magenta: falta mejorar el rango
"""
lm_square = np.full((10, 10, 3), light_magenta, dtype=np.uint8) / 255.0
dm_square = np.full((10, 10, 3), dark_magenta, dtype=np.uint8) / 255.0
plt.subplot(1, 2, 1)
plt.imshow(hsv_to_rgb(dm_square))
plt.subplot(1, 2, 2)
plt.imshow(hsv_to_rgb(lm_square))
plt.show()
"""

#Rangos verde: probados buenos
light_verde = (45, 0, 0)
dark_verde = (80, 100, 255)
#Ver verde
"""
lv_square = np.full((10, 10, 3), light_verde, dtype=np.uint8) / 255.0
dv_square = np.full((10, 10, 3), dark_verde, dtype=np.uint8) / 255.0
plt.subplot(1, 2, 1)
plt.imshow(hsv_to_rgb(dv_square))
plt.subplot(1, 2, 2)
plt.imshow(hsv_to_rgb(lv_square))
plt.show()
"""

#Rangos amarillo: 
light_ama = (15, 0, 0)
dark_ama = (35, 255, 255)
#Ver amarillo
"""
lv_square = np.full((10, 10, 3), light_ama, dtype=np.uint8) / 255.0
dv_square = np.full((10, 10, 3), dark_ama, dtype=np.uint8) / 255.0
plt.subplot(1, 2, 1)
plt.imshow(hsv_to_rgb(dv_square))
plt.subplot(1, 2, 2)
plt.imshow(hsv_to_rgb(lv_square))
plt.show()
"""

#Rangos naranja: 
light_nar = (12, 100, 200)
dark_nar = (16, 255, 255)
#Ver naranja
"""
lv_square = np.full((10, 10, 3), light_nar, dtype=np.uint8) / 255.0
dv_square = np.full((10, 10, 3), dark_nar, dtype=np.uint8) / 255.0
plt.subplot(1, 2, 1)
plt.imshow(hsv_to_rgb(dv_square))
plt.subplot(1, 2, 2)
plt.imshow(hsv_to_rgb(lv_square))
plt.show()
"""

#Rangos azul: 
light_azul = (100, 125, 10)
dark_azul = (120, 255, 255)
#Ver azul
"""
lv_square = np.full((10, 10, 3), light_azul, dtype=np.uint8) / 255.0
dv_square = np.full((10, 10, 3), dark_azul, dtype=np.uint8) / 255.0
plt.subplot(1, 2, 1)
plt.imshow(hsv_to_rgb(dv_square))
plt.subplot(1, 2, 2)
plt.imshow(hsv_to_rgb(lv_square))
plt.show()
"""

#Rangos rojo:
light_rojo = (168, 50, 0)
dark_rojo = (180, 255, 255)
#Ver rojo
"""
lv_square = np.full((10, 10, 3), light_rojo, dtype=np.uint8) / 255.0
dv_square = np.full((10, 10, 3), dark_rojo, dtype=np.uint8) / 255.0
plt.subplot(1, 2, 1)
plt.imshow(hsv_to_rgb(dv_square))
plt.subplot(1, 2, 2)
plt.imshow(hsv_to_rgb(lv_square))
plt.show()
"""

#----------------------------------------MÁSCARAS--------------------------------------------
#Máscara Cyan
#Pone 1 en el píxel que está dentro del rango, sino pone 0
maskcyan = cv2.inRange(hsv_prueba2, light_cyan, dark_cyan)
#Para prueba en toda la visión del aula
maskcyan3 = cv2.inRange(hsv_prueba3, light_cyan, dark_cyan)
#Aplica la máscara
resultcyan = cv2.bitwise_and(prueba2, prueba2, mask=maskcyan)
"""
plt.subplot(1, 2, 1)
plt.imshow(maskcyan, cmap="gray")
plt.subplot(1, 2, 2)
plt.imshow(resultcyan)
plt.show()
"""

#Máscara Magenta
#Pone 1 en el píxel que está dentro del rango, sino pone 0
maskmagenta = cv2.inRange(hsv_prueba2, light_magenta, dark_magenta)
maskmagenta3 = cv2.inRange(hsv_prueba3, light_magenta, dark_magenta)
#Aplica la máscara
resultmagenta = cv2.bitwise_and(prueba2, prueba2, mask=maskmagenta)

plt.subplot(1, 2, 1)
plt.imshow(maskmagenta, cmap="gray")
plt.subplot(1, 2, 2)
plt.imshow(resultmagenta)
plt.show()


#Máscara Verde
#Pone 1 en el píxel que está dentro del rango, sino pone 0
maskverde = cv2.inRange(hsv_prueba2, light_verde, dark_verde)
maskverde3 = cv2.inRange(hsv_prueba3, light_verde, dark_verde)
#Aplica la máscara
resultverde = cv2.bitwise_and(prueba2, prueba2, mask=maskverde)
"""
plt.subplot(1, 2, 1)
plt.imshow(maskverde, cmap="gray")
plt.subplot(1, 2, 2)
plt.imshow(resultverde)
plt.show()
"""

#Máscara Amarilla
#Pone 1 en el píxel que está dentro del rango, sino pone 0
maskama = cv2.inRange(hsv_prueba2, light_ama, dark_ama)
maskama3 = cv2.inRange(hsv_prueba3, light_ama, dark_ama)
#Aplica la máscara
resultama = cv2.bitwise_and(prueba2, prueba2, mask=maskama)
"""
plt.subplot(1, 2, 1)
plt.imshow(maskama, cmap="gray")
plt.subplot(1, 2, 2)
plt.imshow(resultama)
plt.show()
"""

#Máscara Naranja
#Pone 1 en el píxel que está dentro del rango, sino pone 0
masknar = cv2.inRange(hsv_prueba2, light_nar, dark_nar)
masknar3 = cv2.inRange(hsv_prueba3, light_nar, dark_nar)
#Aplica la máscara
resultnar = cv2.bitwise_and(prueba2, prueba2, mask=masknar)
"""
plt.subplot(1, 2, 1)
plt.imshow(masknar, cmap="gray")
plt.subplot(1, 2, 2)
plt.imshow(resultnar)
plt.show()
"""

#Máscara Azul
#Pone 1 en el píxel que está dentro del rango, sino pone 0
maskazul = cv2.inRange(hsv_prueba1, light_azul, dark_azul)
maskazul3 = cv2.inRange(hsv_prueba3, light_azul, dark_azul)
#Aplica la máscara
resultazul = cv2.bitwise_and(prueba1, prueba1, mask=maskazul)
"""
plt.subplot(1, 2, 1)
plt.imshow(maskazul, cmap="gray")
plt.subplot(1, 2, 2)
plt.imshow(resultazul)
plt.show()
"""

#Máscara Rojo
#Pone 1 en el píxel que está dentro del rango, sino pone 0
maskrojo = cv2.inRange(hsv_prueba1, light_rojo, dark_rojo)
maskrojo3 = cv2.inRange(hsv_prueba3, light_rojo, dark_rojo)
#Aplica la máscara
resultrojo = cv2.bitwise_and(prueba1, prueba1, mask=maskrojo)
"""
plt.subplot(1, 2, 1)
plt.imshow(maskrojo, cmap="gray")
plt.subplot(1, 2, 2)
plt.imshow(resultrojo)
plt.show()
"""

#Máscara de todos los colores de identificadores:
maskAll=maskrojo3+maskazul3+maskama3+maskcyan3+maskverde3+maskmagenta3+masknar3
#Aplica la máscara
resultAll = cv2.bitwise_and(prueba3, prueba3, mask=maskAll)
"""
plt.subplot(1, 2, 1)
plt.imshow(maskAll, cmap="gray")
plt.subplot(1, 2, 2)
plt.imshow(resultAll)
plt.show()
"""
