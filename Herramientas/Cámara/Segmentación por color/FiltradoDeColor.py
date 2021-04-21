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
#imagen con montaje
prueba = cv2.imread('PruebaIdentificadoresColor.jpg')
#fotografía de cómo se ven los identificadores ya impresos en las condiciones del aula
prueba2= cv2.imread('realtest1.jpg')
#Imágenes para encontrar los valores HSV aproximados
cyan = cv2.imread('CYAN.jpg')
magenta = cv2.imread('MAGENTA.jpg')
verde = cv2.imread('VERDE.jpg')
#plt.imshow(prueba)
#plt.show()
#Por defecto, OpenCV lee las imágenes en formato BGR, por eso los canales R y B salen
#intercambiados.

#Cambiando a RGB
prueba1 = cv2.cvtColor(prueba, cv2.COLOR_BGR2RGB)
prueba2 = cv2.cvtColor(prueba2, cv2.COLOR_BGR2RGB)
cyan1 = cv2.cvtColor(cyan, cv2.COLOR_BGR2RGB)
magenta1 = cv2.cvtColor(magenta, cv2.COLOR_BGR2RGB)
verde1 = cv2.cvtColor(verde, cv2.COLOR_BGR2RGB)
#ver una imagen en rgb
#plt.imshow(magenta1)
#plt.show()


#Convertir RGB a HSV
hsv_prueba1 = cv2.cvtColor(prueba1, cv2.COLOR_RGB2HSV)
hsv_prueba2 = cv2.cvtColor(prueba2, cv2.COLOR_RGB2HSV)
hsv_cyan1 = cv2.cvtColor(cyan1, cv2.COLOR_RGB2HSV)
hsv_magenta1 = cv2.cvtColor(magenta1, cv2.COLOR_RGB2HSV)
hsv_verde1 = cv2.cvtColor(verde1, cv2.COLOR_RGB2HSV)
#Mostrar en 3D el espacio HSV de la imagen
#Normalizar
#Imagen prueba
"""
pixel_colors = hsv_prueba1.reshape((np.shape(hsv_prueba1)[0]*np.shape(hsv_prueba1)[1], 3))
norm = colors.Normalize(vmin=-1.,vmax=1.)
norm.autoscale(pixel_colors)
pixel_colors = norm(pixel_colors).tolist()

h, s, v = cv2.split(hsv_prueba1)
fig = plt.figure()
axis = fig.add_subplot(1, 1, 1, projection="3d")

axis.scatter(h.flatten(), s.flatten(), v.flatten(), facecolors=pixel_colors, marker=".")
#Hue es un ángulo en grados
axis.set_xlabel("Hue")
axis.set_ylabel("Saturation")
#value es la intensidad luminosa de 0 a 255
axis.set_zlabel("Value")
plt.show()
"""
#Imagen capturada con identificadores impresos

pixel_colors = hsv_prueba2.reshape((np.shape(hsv_prueba2)[0]*np.shape(hsv_prueba2)[1], 3))
norm = colors.Normalize(vmin=-1.,vmax=1.)
norm.autoscale(pixel_colors)
pixel_colors = norm(pixel_colors).tolist()

h, s, v = cv2.split(hsv_prueba2)
fig = plt.figure()
axis = fig.add_subplot(1, 1, 1, projection="3d")

axis.scatter(h.flatten(), s.flatten(), v.flatten(), facecolors=pixel_colors, marker=".")
#Hue es un ángulo en grados
axis.set_xlabel("Hue")
axis.set_ylabel("Saturation")
#value es la intensidad luminosa de 0 a 255
axis.set_zlabel("Value")
plt.show()

#Imagen cyan
"""
pixel_colors = hsv_cyan1.reshape((np.shape(hsv_cyan1)[0]*np.shape(hsv_cyan1)[1], 3))
norm = colors.Normalize(vmin=-1.,vmax=1.)
norm.autoscale(pixel_colors)
pixel_colors = norm(pixel_colors).tolist()

h, s, v = cv2.split(hsv_cyan1)
fig = plt.figure()
axis = fig.add_subplot(1, 1, 1, projection="3d")

axis.scatter(h.flatten(), s.flatten(), v.flatten(), facecolors=pixel_colors, marker=".")
#Hue es un ángulo en grados
axis.set_xlabel("Hue")
axis.set_ylabel("Saturation")
#value es la intensidad luminosa de 0 a 255
axis.set_zlabel("Value")
plt.show()
"""
#Imagen magenta
"""
pixel_colors = hsv_magenta1.reshape((np.shape(hsv_magenta1)[0]*np.shape(hsv_magenta1)[1], 3))
norm = colors.Normalize(vmin=-1.,vmax=1.)
norm.autoscale(pixel_colors)
pixel_colors = norm(pixel_colors).tolist()

h, s, v = cv2.split(hsv_magenta1)
fig = plt.figure()
axis = fig.add_subplot(1, 1, 1, projection="3d")

axis.scatter(h.flatten(), s.flatten(), v.flatten(), facecolors=pixel_colors, marker=".")
#Hue es un ángulo en grados
axis.set_xlabel("Hue")
axis.set_ylabel("Saturation")
#value es la intensidad luminosa de 0 a 255
axis.set_zlabel("Value")
plt.show()
"""
#Imagen verde
"""
pixel_colors = hsv_verde1.reshape((np.shape(hsv_verde1)[0]*np.shape(hsv_verde1)[1], 3))
norm = colors.Normalize(vmin=-1.,vmax=1.)
norm.autoscale(pixel_colors)
pixel_colors = norm(pixel_colors).tolist()

h, s, v = cv2.split(hsv_verde1)
fig = plt.figure()
axis = fig.add_subplot(1, 1, 1, projection="3d")

axis.scatter(h.flatten(), s.flatten(), v.flatten(), facecolors=pixel_colors, marker=".")
#Hue es un ángulo en grados
axis.set_xlabel("Hue")
axis.set_ylabel("Saturation")
#value es la intensidad luminosa de 0 a 255
axis.set_zlabel("Value")
plt.show()
"""
#Rangos cyan
light_cyan = (90, 255, 255)
dark_cyan = (90, 150, 125)
#Rangos magenta
light_magenta = (142, 100, 220)
dark_magenta = (144, 25, 255)
#Rangos verde
light_green = (60, 255, 115)
dark_green = (60, 125, 255)
#Ver los colores elegidos
#cyan
lc_square = np.full((10, 10, 3), light_cyan, dtype=np.uint8) / 255.0
dc_square = np.full((10, 10, 3), dark_cyan, dtype=np.uint8) / 255.0
#Ver cyan
"""
plt.subplot(1, 2, 1)
plt.imshow(hsv_to_rgb(dc_square))
plt.subplot(1, 2, 2)
plt.imshow(hsv_to_rgb(lc_square))
plt.show()
"""
#Ver magenta: falta mejorar el rango
lm_square = np.full((10, 10, 3), light_magenta, dtype=np.uint8) / 255.0
dm_square = np.full((10, 10, 3), dark_magenta, dtype=np.uint8) / 255.0
"""
plt.subplot(1, 2, 1)
plt.imshow(hsv_to_rgb(dm_square))
plt.subplot(1, 2, 2)
plt.imshow(hsv_to_rgb(lm_square))
plt.show()
"""

#Ver verde
lv_square = np.full((10, 10, 3), light_green, dtype=np.uint8) / 255.0
dv_square = np.full((10, 10, 3), dark_green, dtype=np.uint8) / 255.0
"""
plt.subplot(1, 2, 1)
plt.imshow(hsv_to_rgb(dv_square))
plt.subplot(1, 2, 2)
plt.imshow(hsv_to_rgb(lv_square))
plt.show()
"""

#Rangos prueba: funciona
light_cyan = (0, 100, 0)
dark_cyan = (180, 255, 255)
#Máscara
#Pone 1 en el píxel que está dentro del rango, sino pone 0
maskcyan = cv2.inRange(hsv_prueba2, light_cyan, dark_cyan)
#Aplica la máscara
resultcyan = cv2.bitwise_and(prueba2, prueba2, mask=maskcyan)
plt.subplot(1, 2, 1)
plt.imshow(maskcyan, cmap="gray")
plt.subplot(1, 2, 2)
plt.imshow(resultcyan)
plt.show()
