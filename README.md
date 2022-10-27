# PROE
PROYECTO PROE: Implementación de un prototipo de enjambre de robots para la digitalización de escenarios estáticos y planificación de rutas óptimas.

Proyecto de investigación del Instituto Tecnológico de Costa Rica, dirigido por la Escuela de Matemática y con colaboración del Área Académica de Ingeniería Mecatrónica.

Este proyecto pretende diseñar un sistema multirobot físico para la exploración de escenarios estáticos desconocidos, con el fin de determinar rutas óptimas. Dicha implementación significa una traducción y adaptación de los algoritmos computacionales de exploración investigados en el proyecto “PROE: Simulación computacional para la planificación de rutas óptimas de acceso y/o evacuación por medio de un enjambre centralizado en escenarios estáticos, utilizando técnicas de mapeo, procesamiento de datos y optimización multiobjetivo” (PROE E1F1 ); además de implementar técnicas de recolección, transferencia y procesamiento de datos para mapear la zona y así poder determinar rutas de acceso y/o evacuación seguras.

Los códigos computacionales, archivos CAD, diagramas eléctricos, esquemas de ensamblaje, lista de materiales y demás documentación acá presente representan el esfuerzo llevado acabo en este proyecto.

Directorio de archivos:
- Arquitectura_PROE-> archivo PDF de la arquitectura del sistema electrónico
- Corte_Chasis_PROE -> archivo PDF para corte láser del chasís 
- Corte_Obstaculos_PROE -> archivo PDF para corte láser de los obstáculos del escenario 
- PCB_Conexiones_PROE-> archivo PDF con la conexiones del PCB
- BOM_PROE -> archivo con la lista de materiales para el proyecto PROE
- EstandarCodigo -> Lineamientos para escritura de código en el proyecto
- Readme 
- LICENSE

Directorio de carpetas:
- PROE_LOCOM -> Proyecto en Arduino IDE (placa Feather M0) para Locomoción y Comunicación
- PROE_DETOB -> Proyecto en Arduino IDE (placa Bluepill STM32) para Detección de Obstáculos
- PROE_COMBASE -> Proyecto en Arduino IDE (placa Feather M0) dispositivo usado como base para la comunicación
- Chasis: archivos CAD para corte láser del chasís
- Obstáculos: archivos CAD para corte láser de los obstáculos
- PCB: archivos CAD para diseño del PCB y archivos Gerber para manufactura del PCB
- Herramientas: archivos para calibración y validación
	- Vision_Validacion -> herramienta de Matlab para validar giros y desplazamientos de un robot
	- Vision_Patron -> archivo PDF del patrón que debe aparecer en las imágenes para calibrar
	- Vision_Guia -> archivo PDF de la figura que debe ubicarse en cada robot para seguimiento
	- Calibracion_Motores -> archivo para correr en feather y calibrar los motores
	- Calibración_Magnetómetro -> archivo para correr en feather y calibrar el magnetómetro


Investigadores: 
- Cindy Calderón Arce ccalderon@itcr.ac.cr (Directora del proyecto)
- Rebeca Solís Ortega rsolis@itcr.ac.cr
- Juan Carlos Brenes-Torres juanbrenes@tec.ac.cr

Asistentes:
- Bryan Alpízar Quesada bryan2300@gmail.com
- Andrés Jiménez Mora andjm1428@gmail.com
- Célimo Porras Aguilar celimojosepa@estudiantec.cr
- David Gómez Carrasquilla davidjgomezc@gmail.com
