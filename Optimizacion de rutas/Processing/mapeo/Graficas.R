library(RColorBrewer)
library(scales)

# Data frames que se leen de los archivos de datos
principal <- read.csv("Principal.csv")
resumen   <- read.csv("Resumen.csv")

# Paleta de colores
colors <- brewer.pal(4,"Set2")

#####################################################################

# PRIMERA GRÁFICA

# Filtro los datos para extraer solo los de percentil 100
principal_filtrado <- subset(principal, Percentil == 100)

# Separo los datos por duración
split_data <- split(principal_filtrado, principal_filtrado$Duracion)

#Ventana gráfica
windows(width = 14, height=8)   # Tamaño de la ventana
par(mar = c(5, 5, 4, 6))        # Márgenes del gráfico
par(mfrow = c(1, 3))            # Organizo los gráficos en 3 columnas

tiempos <- c(5,10,15)

# En la ventana grafico los mismos datos pero para cada duración
for (i in 1:3) {
  # Scatter plot
  plot(split_data[[i]]$Distancia, split_data[[i]]$Peligrosidad,
       col=colors[split_data[[i]]$Robots], # Color según robots
       cex=2,   # Tamaño de puntos
       pch=19,  # Forma de los puntos (19 es círculo relleno)
       xlab = "Distancia", ylab = "Peligrosidad", # Títulos de ejes
       ylim = c(0,31),   # Límites eje y
       xlim = c(0,1600), # Límites eje x
       main = paste("D vs P en cuartil 100% para", tiempos[[i]],"minutos.")) # Título
  
  # Leyenda
  legend("topleft",inset=0.01,                  # Posición de leyenda y offset
         legend=unique(split_data[[i]]$Robots), # Leyenda respecto a robots
         fill = colors,                         # Colores
         xpd=TRUE,
         title="Cantidad\nde robots")           # Título de la leyenda
}

#####################################################################

# SEGUNDO GRÁFICO

# Filtro el dataset RESUMEN para la Duración de 15 minutos y la Métrica %Cobertura
data_filtered <- subset(resumen, Duracion == 15 & Metrica == "%Cobertura")

# A 1 Robot le resto 5% a los percentiles y a 3 Robots les sumo 5% a los percentiles
data_filtered[data_filtered$Robots == 1, "Percentil"] <- data_filtered[data_filtered$Robots == 1, "Percentil"] - 5
data_filtered[data_filtered$Robots == 3, "Percentil"] <- data_filtered[data_filtered$Robots == 3, "Percentil"] + 5

# Filtro TODAS las pruebas para la duración de 15 minutos
datos_nube <- subset(principal, Duracion == 15)
datos_nube[datos_nube$Robots == 1, "Percentil"] <- datos_nube[datos_nube$Robots == 1, "Percentil"] - 5
datos_nube[datos_nube$Robots == 3, "Percentil"] <- datos_nube[datos_nube$Robots == 3, "Percentil"] + 5

#Ventana gráfica
windows(width = 14, height=8)
par(mar = c(5, 5, 4, 6))

# Scatterplot
plot(data_filtered$Percentil, data_filtered$Promedio,
     col=colors[data_filtered$Robots], 
     cex=1.2,
     pch=19,
     xlab = "Porcentaje de tiempo transcurrido", ylab = "Porcentaje de cobertura",
     ylim = c(-5,100),
     xlim = c(20,110),
     lwd=5,             # Grosor de puntos (para que destaquen)
     xaxt = 'n',        # Quito eje x para hacerlo custom luego
     main = paste("Dispersión y error de porcentaje de cobertura según el tiempo."))

# Agrego el eje x que yo quiero
axis(1, at = c(25, 50, 75, 100), labels = c(25, 50, 75, 100))

# Leyenda
legend("topleft",inset=0.01,
       legend=unique(split_data[[i]]$Robots),
       fill = colors,
       xpd=TRUE,
       title="Cantidad\nde robots")

# Agrego barras de error restando y sumando la desviación estándar al promedio
arrows(data_filtered$Percentil, data_filtered$Promedio-data_filtered$DesvEst, data_filtered$Percentil, 
       data_filtered$Promedio+data_filtered$DesvEst,
       col=colors[data_filtered$Robots],
       length=0.07,  # Longitud de las flechas en los extremos de las barras
       angle=90,     # Ángulo de las flechas, 90° para que sean horizontales
       lwd=3,        # Grosor de línea (para que destaque)
       code=3)

# Agrego nube de puntos de TODAS las pruebas
points(datos_nube$Percentil, datos_nube$X.Cobertura,
       col=alpha(colors[data_filtered$Robots], .2),  # Los coloreo pero los hago transparentes con el segundo parámetro
       pch=19,
       cex=5    # Tamaño de los puntos
)

#####################################################################

# TERCER GRÁFICO
# Reorganizo datos pues necesito las diferencias de coberturas entre cuartiles
diferencias <- matrix(0, nrow=4, ncol=3)
for(robot in unique(data_filtered$Robots)) {
  cobertura <- data_filtered[data_filtered$Robots == robot, "Promedio"]
  diferencias[, robot] <- c(
    cobertura[1],
    cobertura[2] - cobertura[1],
    cobertura[3] - cobertura[2],
    cobertura[4] - cobertura[3]
  )
}
# Le pongo nombre a las filas y columnas de la nueva matriz
rownames(diferencias)  <- c('25%','50%','75%','100%')
colnames(diferencias)   <- c(1, 2, 3)

#Ventana gráfica
windows(width = 10, height=8)
par(mar = c(5, 5, 4, 6))

# Grafica de barras
barplot(diferencias, 
        main = "Porcentaje de cobertura por cantidad de robots",
        xlab = "Cantidad de robots",
        ylab = "% Cobertura",
        col = colors,    # Aqui los colores no van respecto a los robots
        ylim = c(0,100))

# Leyenda
legend("top",
       rownames(diferencias),  # Si recordamos los nombres de las filas son ('25%','50%','75%','100%')
       fill = colors,
       xpd=TRUE,
       horiz=TRUE)             # Para que la leyenda sea horizontal

#####################################################################