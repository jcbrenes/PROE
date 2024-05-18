//Código de locomoción y comunicacion para el proyecto PROE
//Usado en la placa Feather M0 RFM95
//https://github.com/jcbrenes/PROE

#include <Wire.h> //Bilbioteca para la comunicacion I2C
#include "wiring_private.h" // Necesario para el I2C secundario y usar pinPeripheral() function
#include <SPI.h> //Biblioteca para la comunicacion por radio frecuencia
#include <RH_RF69.h> //Biblioteca para la comunicacion por radio frecuencia
#include <RHDatagram.h> // Biblioteca para la comunicación con direcciones

/************ Serial Setup ***************/
#define debug 0

#if debug == 1
#define serialPrint(x) Serial.print(x)
#define serialPrintln(x) Serial.println(x)
#else
#define serialPrint(x)
#define serialPrintln(x)
#endif

//Variables del enjambre para la comunicación a la base
const uint8_t cantidadRobots = 3; //Cantidad de robots en enjambre. No cuenta la base, solo los que hablan.
unsigned long idRobot = 2; //ID del robot, este se usa para ubicar al robot dentro de todo el ciclo de TDMA.

// Variables para el proceso de arranque y transformación de coordenadas
int distCoordenadas; //Distancia inicial de los robot 
bool datoCoordenadas = false; // Primer dato STM32 es la distancia entre robots
int matTransformation[int(cantidadRobots)]; // Arreglo utilizao para transformar las coordenadas de los robots.
int cantPaquetesDistanciaEnviados = 0; //Variable para contar la cantidad de paquetes que envia con la medición inicial
int cantPaquetesDistanciaPorEnviar = 5; // Cantidad de paquetes a enviar para la medición inicial.
int posicionXAvanzarCoordenada = 0; // Posicion X a la que va a avanzar el robot en la funcion avanzarCoordenada
int posicionYAvanzarCoordenada = 0; // Posicion Y a la qu va a avanzar el robot en la funcion avanzarCoordenada
int errorAvanzarCoordenada = 30; // Error que se permita de la posicion XY de la funcion avanzarCoordenada, debe ser minimo el ancho de los Atta

// Variables recibidas por los otros Atta-Bots
int16_t posXRecibido;
int16_t posYRecibido;
int16_t rotRecibido;
int8_t tipSensRecibido;
int16_t disRecibido;
int16_t anguloRecibido;

//constantes del robot empleado
const int tiempoMuestreo=100000; //unidades: micro segundos
const float pulsosPorRev=206.0; //cantidad de pulsos de una única salida
const int factorEncoder=2; //cantidad de salidas de encoders que se están detectando (pueden ser 2 o 4)
const float circunferenciaRueda=139.5;//Circunferencia de la rueda = 139.5mm 
const float pulsosPorMilimetro=((float)factorEncoder*pulsosPorRev)/circunferenciaRueda; 
const float distanciaCentroARueda=63.7;// Radio de giro del carro, es la distancia en mm entre el centro y una rueda. 
const float conversionMicroSaMin=1/(60 * 1000000);// factor de conversion microsegundo (unidades del tiempo muestreo) a minuto
const float conversionMicroSaSDiv=1000000;// factor de conversion microsegundo (unidades del tiempo muestreo) a segundo
const float tiempoMuestreoS= (float)tiempoMuestreo/conversionMicroSaSDiv;

//constantes para control PID movimiento lineal para tiempoMuestreo=100000 us
const float velRequerida=120.0; //unidades mm/s
const float KpVel=0.4; //constante control proporcional
const float KiVel=0.65; //constante control integral
const float KdVel=0.0; //constante control derivativo

//constantes para control PID de giro 
const float KpGiro=3.0; //constante control proporcional
const float KiGiro=1.1;//constante control integral
const float KdGiro=0.17; //constante control derivativo

//constantes para control PID de la pose del robot 
const float KpPose=1.0; //constante control proporcional
const float KdPose=0.0; //constante control derivativo
const float KiPose=0.0; //constante control derivativo
const float velBase=120.0; //unidades mm/s

//Constantes para la implementación del control PID real
const int errorMinIntegralVelocidad=-255;
const int errorMaxIntegralVelocidad=255;
const int errorMinIntegralGiro=-110;
const int errorMaxIntegralGiro=110;
const int limiteSuperiorCicloTrabajoVelocidad=200;
const int limiteInferiorCicloTrabajoVelocidad=-200;
int limiteSuperiorCicloTrabajoGiro=110;
int limiteInferiorCicloTrabajoGiro=-110;
const int limiteSuperiorCicloTrabajoGiroCalibracion=50;
const int limiteInferiorCicloTrabajoGiroCalibracion=-50;
const int cicloTrabajoMinimo= 20;
const int minCiclosEstacionario= 20;
const float correcionVelRuedas = 0.5;

//Configuración de pines de salida para conexión con el Puente H
const int PWMA = 12; //Control velocidad izquierdo
const int AIN1 = 10; //Dirección 1 rueda izquierda
const int AIN2 = 1; //Dirección 2 rueda izquierda
const int PWMB = 5;  //Control velocidad derecha
const int BIN1 = 9;  //Dirección 1 rueda derecha
const int BIN2 = 6;  //Dirección 2 rueda derecha

//Configuración de los pines para la conexión con los Encoders
const int ENC_DER_C1 =  A0; //Encoders rueda derecha
const int ENC_DER_C2 =  A1;
const int ENC_IZQ_C1 =  A2; //Encoders rueda izquierda
const int ENC_IZQ_C2 =  A3;

//Configuracion de los pines de interrupcion de Obstáculos
const int INT_OBSTACULO = A4;

//Almacenamiento de datos de obstáculos
const int longitudArregloObstaculos = 100;

//Constantes algoritmo exploración
const int unidadAvance= 1000; //medida en mm que avanza cada robot por movimiento
const int unidadRetroceso= -50; //medida en mm que retrocede el robot al encontrar un obstáculo
const unsigned long limiteRetroceso= 5000; //Máximo tiempo (ms) permitido para completar el retroceso, si no termina en ese tiempo asume que tiene un obstaculo atras
unsigned long tiempoRetroceso=0; //Almacena el momento en que se cambia a estado de retroceso para usar el limite de tiempo
const int limiteGiro= 5000; //Máximo tiempo permitido para completar el giro, permite que siga el movimiento si no se puede completar el giro por obstaculo o no se logra estado estacionario
unsigned long tiempoGiro=0; //Almacena el momento en que se cambia al estado de giro

//Constantes comunicación I2C con STM
const int dirSTM= 8; 
const int cantBytesMsg= 10; //cantidad de bytes que se solicitan al STM
//Definición para usar el segundo puerto I2C
TwoWire segundoI2C(&sercom1, 11, 13);

//Constantes bus I2C periféricos
#define dirEEPROM B01010000 //Direccion de la memoria EEPROM
#define dirMag 0x0D //I2C Address para el HMC5883
//Radios de conversión según data sheet del MPU6050
#define A_R 16384.0
#define G_R 131.0

//VARIABLES GLOBALES

//Variables para la máquina de estados principal
enum PosiblesEstados {AVANCE=0, RETROCEDA, GIRE_DERECHA, GIRE_IZQUIERDA, ESCOGER_DIRECCION, GIRO, CONTROL_POSE, NADA};
//char const *PosEstados[] = {"AVANCE", "RETROCEDA", "GIRE_DERECHA", "GIRE_IZQUIERDA","ESCOGER_DIRECCION","GIRO", "NADA"};
PosiblesEstados estado = AVANCE;

bool banderaParar = false; // Detiene todo avance del robot


//variable que almacena el tiempo del último ciclo de muestreo
unsigned long tiempoActual = 0;
unsigned long tiempoMaquinaEstados = 0;
unsigned long tiempoPollingPeri = 0;

//contadores de pulsos del encoder
int contPulsosDerecha = 0;
int contPulsosIzquierda = 0;
int contPulsosDerPasado = 0;
int contPulsosIzqPasado = 0;
static int8_t tablaEncoder[] = {0,0,0,-1,0,0,1,0,0,1,0,0,-1,0,0,0};

//Variables que almacenan el desplazamiento angular de cada rueda
float posActualRuedaDerecha = 0.0;
float posActualRuedaIzquierda = 0.0;

//Variables del valor de velocidad en cada rueda
float velActualDerecha = 0.0;
float velActualIzquierda = 0.0;

//Valores acumulados para uso en las ecuaciones de control
//Para Giro
float errorAnteriorGiroDer = 0;
float errorAnteriorGiroIzq = 0;
float sumErrorGiroDer = 0;
float sumErrorGiroIzq = 0;
int contCiclosEstacionarioGiro = 0;
//Para Velocidad
float errorAnteriorVelDer = 0;
float errorAnteriorVelIzq = 0;
float sumErrorVelDer = 0;
float sumErrorVelIzq = 0;

//Para Coordenadas
float errorAnt_1_Orientacion = 0;
float errorAnt_2_Orientacion = 0;
float sumErrorOrientacion = 0;
float distLinealRuedaDerecha = 0;
float distLinealRuedaIzquierda = 0;
float distLinealRuedaDerechaAnterior = 0;
float distLinealRuedaIzquierdaAnterior = 0;
bool finPose = false;
bool corregirOrientacion = false;

//Variables para las interrupciones de los encoders
byte estadoEncoderDer = 1;
byte estadoEncoderIzq = 1;

//Variables para el almacenar datos de obstáculos
int datosSensores[longitudArregloObstaculos][6] = {0}; //Arreglo que almacena la información de obstáculos de los sensores (tipo sensor, distancia, ángulo)
int ultimoObstaculo = -1; //Apuntador al último obstáculo en el arreglo. Se inicializa en -1 porque la función de guardar aumenta en 1 el índice
int minEnviado = 0; //Almacena el recorrido de la lista de obstaculos
bool nuevoObstaculo = false; //Bandera verdadera cuando se detecta un nuevo obstaculo
unsigned long tiempoSTM = 0; //Almacena el tiempo de muestreo del STM
int limiteObstaculos=unidadAvance/10; //Distancia minima que el robot debe moverse para olvidar los obstaculos detectados hasta ese momento, para random walk con memoria
bool resetObstaculos=false; //Bandera que permite reiniciar los obstaculos detectados en loop principal para no afectar tiempos de movimiento

//Variables para el algoritmo de exploración
int distanciaAvanzada = 0;
float poseActual[3] = {0, 0, 0}; //Almacena la pose actual: ubicación en x, ubicación en y, orientación.
float anguloOrientacion = 0; // Variable que guarda la orientacion actual del robot en cada momento
float tempOrientacionGiroscopio = 0; // Variable para obtener la orientacion del giroscopio
float avanceRealizado = 0; // Avance que el robot realiza en cada intervalo de tiempo
float errorOrientacion = 0; // Se utiliza para la funcion avanzar coordenada
bool giroTerminado = 1; //Se hace esta variable global para saber cuando se está en un giro y cuando no
bool obstaculoAdelante = false;
bool obstaculoDerecha = false;
bool obstaculoIzquierda = false;
bool obstaculoAtras = false;
bool sinObstaculo = false;
enum orientacionesRobot {ADELANTE = 0, DERECHA = 90, IZQUIERDA = -90, ATRAS = 180}; //Se definen las orientaciones de los robots, el número indica la orientación de la pose
orientacionesRobot direccionGlobal = ADELANTE; //Se inicializa Adelante, pero en Setup se asignará un valor random
int anguloGiro = 0;

//Variables para el magnetómetro y su calibración
const float declinacionMag = 0.0; //correccion del campo magnetico respecto al norte geográfico en Costa Rica
const float alfa = 0.8; //constante para filtro de datos
short x, y, z;  //Valores crudos del magnetometro
float xof, yof, xrot, yrot, xf, yf; //Variables de funcion medirMagnet
int cuentaMagnetError=0; //Cuenta de error del magnetometro
bool magnetError=false; //Bandera de error de magnetometro
float xft=0; //Valores filtrados
float yft=0;
float xoff = 0; //offset de calibración en x
float yoff = 0; //offset de calibración en y
float angElips = 0; //angulo del elipsoide que forman los datos
float factorEsc = 1; //factor para convertir el elipsoide en una circunferencia
float magInicioOrigen=0; //valor de orientación inicial
float orientacion=0; //Usado en ActualizarUbicacion


//Variables para el MPU6050 y su calibración
unsigned long tiempoPrev = 0;
float gir_ang_zPrev, vel_y_Prev; //angulos previos para determinar el desplazamiento angular
float gx_off, gy_off, gz_off, acx_off, acy_off, acz_off; // offsets para calibración del MPU6050
float ayft, azft, gzft;
int16_t gyro_x, gyro_y, gyro_z, tmp, ac_x, ac_y, ac_z; //guardan los datos crudos del MPU
float gz, ay; //guardan los valores reales de aceleración y velocidad angular
long dt; //delta de tiempo para calcular desplazamiento angular
int cuentaMPUerror = 0;  //Cuenta de error del MPU
bool mpuError = false;  //Bandera de error del MPU
float filtroMPU = 0.90;

//Variables de calibración para magnetometro 
const int numSamples=2000;
const int desfase=(1-alfa)*20;
short puntosCalibracion=0; //Contadores para calibración
short maxX=0,minX=0,maxY=0,minY=0; // Maximos y minimos de los datos
float uXX=0,uYY=0,uXY=0; //Momentos de inercia
float angulo=0; //Angulo de rotación de los datos

// Filtro de kalman
const float R = 80; // Valor R filtro de Kalman
const float H = 1.00; // Valor de ganancia filtro de Kalman
const float Q = 5; // Valor Q filtro de Kalman
static float P = 0; // Valor de ajuste de Kalman
static float K = 0; // Valor ganancia K de Kalman

bool cal_mag=0;
bool cal_mpu=0;

//Variables para la comunicación por radio frecuencia
#define RF69_FREQ      915.0   //La frecuencia debe ser la misma que la de los demas nodos.
#define DEST_ADDRESS   10      //No sé si esto es totalmente necesario, creo que no porque nunca usé direcciones.
#define MY_ADDRESS     idRobot //Dirección de este nodo. La base la usa para enviar el reloj al inicio

//Definición de pines. Creo que no todos se están usando, me parece que el LED no.
#define RFM69_CS       8
#define RFM69_INT      3
#define RFM69_RST      4

RH_RF69 rf69(RFM69_CS, RFM69_INT); // Inicialización del driver de la biblioteca Radio Head.

// Class to manage message delivery and receipt, using the driver declared above
RHDatagram rf69_manager(rf69, MY_ADDRESS);

//Variables para el mensaje que se va a transmitir a la base
bool timeReceived = false; //Bandera para saber si ya recibí el clock del máster.
unsigned long tiempoRobotTDMA = 30; //Slot de tiempo que tiene cada robot para hablar. Unidades ms.
unsigned long tiempoCicloTDMA = cantidadRobots * tiempoRobotTDMA; //Duración de todo un ciclo de comunicación TDMA. Unidades ms.
unsigned long timeStamp1; //Estampa de tiempo para eliminar el desfase por procesamiento del reloj al recibirse. Se obtiene apenas se recibe el reloj.
unsigned long timeStamp2; //Estampa de tiempo para eliminar el desfase por procesamiento del reloj al recibirse. Se obtiene al guardar en memoria el reloj.
unsigned long data; //Variable donde se almacenará el valor del reloj del máster.
volatile bool mensajeCreado = false; //Bandera booleana para saber si ya debo crear otro mensaje cuando esté en modo transmisor.
const uint8_t timeOffset = 2; //Offset que existe entre el máster enviando y el nodo recibiendo.

uint8_t buf[RH_RF69_MAX_MESSAGE_LEN];
uint8_t len = sizeof(buf);
uint8_t idMensajeRecibido;
uint16_t* ptrMensaje; //Puntero para descomponer datos en bytes.
uint8_t mensaje[5 * sizeof(uint16_t) + sizeof(uint8_t)]; // Mensaje a enviar a la base

//Variables para calcular el giro real
float anguloInicial;
float ultimoAngMagnet;
float anguloMPU;
float velMPU;
float angActualRobot;

//Variables para medir el desplazamiento angular con el magnetómetro
int cuadranteActual;
int cuadranteAnterior;
bool complemento=false;
bool cambio12=false;
bool cambio21=false;

int c = 0; //Contador para led 13
int b = 0;

void setup() {
  //asignación de pines
  pinMode(PWMA, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  //pinMode(INT_OBSTACULO, INPUT_PULLUP);
  pinMode(INT_OBSTACULO,OUTPUT); // Se puede borrar
  pinMode(ENC_DER_C1, INPUT); 
  pinMode(ENC_DER_C2, INPUT); 
  pinMode(ENC_IZQ_C1, INPUT);
  pinMode(ENC_IZQ_C2, INPUT);

   //asignación de interrupciones
  delay(300); //delay para evitar interrupciones al arrancar
  //attachInterrupt(ENC_DER_C1, PulsosRuedaDerechaC1,CHANGE);  //conectado el contador C1 rueda derecha
  attachInterrupt(ENC_DER_C2, PulsosRuedaDerechaC2_2,CHANGE);
  //attachInterrupt(ENC_IZQ_C1, PulsosRuedaIzquierdaC1,CHANGE);  //conectado el contador C1 rueda izquierda
  attachInterrupt(ENC_IZQ_C2, PulsosRuedaIzquierdaC2_2,CHANGE);
  attachInterrupt(digitalPinToInterrupt(INT_OBSTACULO), DeteccionObstaculo,FALLING);
  
  //temporización y variables aleatorias
  tiempoActual=millis(); //para temporización de los ciclos
  tiempoMaquinaEstados= micros();
  tiempoPollingPeri= micros();
  randomSeed(analogRead(A5)); //Para el algoritmo de exploración, el pinA5 está al aire
  direccionGlobal = (orientacionesRobot)(random(-1, 2) * 90); //Se asigna aleatoriamente una dirección global a seguir por el algoritmo RWD
  
  //***Inicialización de puertos seriales***
  if (debug == 1){
    Serial.begin(115200); //Puerto serie para comunicación con la PC
  }
  Wire.begin(); //Puerto I2C para hablar con el STM32. Feather es master
  Wire.setClock(400000); //Velocidad del bus en Fast Mode
  segundoI2C.begin();  //Puerto I2C para comunicarse con los sensores periféricos (Mag, IMU, EEPROM)
  segundoI2C.setClock(400000); //Velocidad del bus en Fast Mode
  pinPeripheral(11, PIO_SERCOM); //Configura los pines para comunicación (por default estan en PWM)
  pinPeripheral(13, PIO_SERCOM);

  //Inicializacion del Magnetómetro
  inicializaMagnet(); 
  resetVarGiro();
  
  //Inicializacion del MPU
  inicializarMPU();

  
  //***Inicialización de RFM69***
  pinMode(RFM69_RST, OUTPUT);
  digitalWrite(RFM69_RST, LOW);

  // Reseteo manual del RF
  digitalWrite(RFM69_RST, HIGH);
  delay(10);
  digitalWrite(RFM69_RST, LOW);
  delay(10);

  if (!rf69_manager.init()) {
    serialPrintln("RFM69 inicialización fallida");
    while (1);
  }
  // Setear frecuencia
  if (!rf69.setFrequency(RF69_FREQ)) {
    serialPrintln("setFrequency failed");
  }

  rf69.setTxPower(20, true); // Configura la potencia
  
  rf69.setModeRx();

  /****Inicialización del RTC****/
  PM->APBAMASK.reg |= PM_APBAMASK_RTC;  //Seteo la potencia para el reloj del RTC.
  configureClock();                     //Seteo reloj interno de 32kHz.
  RTCdisable();                         //Deshabilito RTC. Hay configuraciones que solo se pueden hacer si el RTC está deshabilitado, revisar datasheet.
  RTCreset();                           //Reseteo RTC.

  RTC->MODE0.CTRL.reg = 2UL;            //Esto en binario es 0000000000000010. Esa es la configuración que ocupo en el registro. Ver capítulo 19 del datasheet.
  while (RTCisSyncing());               //Llamo a función de escritura. Hay bits que deben ser sincronizados cada vez que se escribe en ellos. Esto lo hice por precaución..
  RTC->MODE0.COUNT.reg = 0UL;           //Seteo el contador en 0 para iniciar.
  while (RTCisSyncing());               //Llamo a función de escritura.
  RTC->MODE0.COMP[0].reg = 600000UL;    //Valor inicial solo por poner un valor. Más adelante, apenas reciba el clock del master, ya actualiza este valor correctamente. ¿Por qué debo agregar el [0]? Esto nunca lo entendí.
  while (RTCisSyncing());               //Llamo a función de escritura.

  RTC->MODE0.INTENSET.bit.CMP0 = true;  //Habilito la interrupción.
  RTC->MODE0.INTFLAG.bit.CMP0 = true;   //Remover la bandera de interrupción.

  NVIC_EnableIRQ(RTC_IRQn);             //Habilito la interrupción del RTC en el "Nested Vestor
  NVIC_SetPriority(RTC_IRQn, 0x00);     //Seteo la prioridad de la interrupción. Esto se debe investigar más.
  RTCenable();                          //Habilito de nuevo el RTC.
  RTCresetRemove();                     //Quito el reset del RTC.

  //Serial.println("¡RFM69 radio en funcionamiento!");
  delay(1000);

  leerMsgSTM();

  //******En caso de usar el robot solo (no como enjambre), comentar las dos siguiente lineas
  sincronizacion(); //Esperar mensaje de sincronizacion de la base antes de moverse
  // Crear el arreglo para la transformacion de coordenadas
  for (int i = 0; i < cantidadRobots; i++) // Inicializacion del arreglo
  {
    matTransformation[i] = 0;
  }
  transformation(); // Proceso de transformación de coordenadas, comentar en caso de no requerirse

  //descomentar la siguiente linea si se quiere llevar el robot a cierta coordenada
  //estado = CONTROL_POSE;
}

void loop(){
  //***POLLING*** Acciones que se ejecutan periodicamente. Más frecuentemente que la máquina de estados
  RecorrerObstaculos();

  if (rf69_manager.available() && estado != CONTROL_POSE){ // Cuando los robot se agrupan evita que reciba mensajes
    if (rf69_manager.recvfrom(buf, &len, &idMensajeRecibido)){
      buf[len] = 0;
      posXRecibido = *(int16_t*)&buf[0];
      posYRecibido = *(int16_t*)&buf[2];
      rotRecibido = *(int16_t*)&buf[4];
      tipSensRecibido = (int8_t)buf[6];
      disRecibido = *(int16_t*)&buf[7];
      anguloRecibido = *(int16_t*)&buf[9];
      serialPrint(idMensajeRecibido);serialPrint("; ");serialPrint(posXRecibido);serialPrint("; ");serialPrint(posYRecibido);
      serialPrint("; ");serialPrint(rotRecibido);serialPrint("; ");serialPrint(tipSensRecibido);
      serialPrint("; ");serialPrint(disRecibido);serialPrint("; ");serialPrintln(anguloRecibido);
    }
    if(tipSensRecibido == 5){ // Activa que los robot se agrupen, en caso de no ocuparse comentar
      posicionXAvanzarCoordenada = posXRecibido;
      posicionYAvanzarCoordenada = posYRecibido + matTransformation[idMensajeRecibido];
      estado = CONTROL_POSE;
    }
  }

  //***MAQUINA DE ESTADOS*** Donde se manejan los comportamientos del robot y el control PID
  //Las acciones de la máquina de estados y los controles se efectuarán en tiempos fijos de muestreo
  if (banderaParar){ // Detiene el robot por completo
    estado = NADA;
    ConfiguracionParar();
  }
  if((micros()-tiempoMaquinaEstados)>=tiempoMuestreo){
    tiempoMaquinaEstados=micros();
    tiempoActual=millis();//tomo el tiempo actual (para reducir las llamadas a millis)

    //Polling de perifericos conectados a los buses I2C
    leerMsgSTM(); //lectura de obstáculo en el STM
    ultimoAngMagnet = medirMagnet(); //medición de la orientación con el magnetómetro
    serialPrintln(ultimoAngMagnet);
    detectaCambio(ultimoAngMagnet); //detectar cambio de orientación para corrección en valores
    leeMPU(anguloMPU,velMPU); //medición del MPU

    //Pruebas de perifericos conectados a los buses I2C
    if(magnetError){
      ConfiguracionParar();
      CrearObstaculo(40,0,0);
      Serial.println("Magnet Error");
      return;
    }

    if(mpuError){
      ConfiguracionParar();
      CrearObstaculo(41,0,0);
      Serial.println("MPU Error");
      return;
    }

    actualizarUbicacion(); // Actualiza constantemente las coordenadas XY del robot

    //Máquina de estados que cambia el modo de operación
    switch (estado) {

      case AVANCE:  { 
        bool avanceTerminado = AvanzarDistancia(unidadAvance); 
        //bool avanceTerminado= true;
        if (avanceTerminado){
          ConfiguracionParar(); 
          CrearObstaculo(0,0,0); //Si completa el avance crea un obstaculo 0 para indicar que no encontro obstaculos
          estado = ESCOGER_DIRECCION;
          //estado = RETROCEDA; 
        }        
        break;
      }

      case RETROCEDA:  { 
        bool retrocesoTerminado= AvanzarDistancia(unidadRetroceso); 

        if ((tiempoActual - tiempoRetroceso) > limiteRetroceso){ //Si pasa mas del limite de tiempo tratando retroceder detiene el retroceso, probablemente esté bloqueado y no tiene sensores atras
          retrocesoTerminado = true;
        }
        
        if (retrocesoTerminado){
          ConfiguracionParar();
          estado = ESCOGER_DIRECCION; 
        }   
        break; 
      }
      
      case GIRE_DERECHA: { 
        giroTerminado= Giro(90);
        if   (giroTerminado) {
          ConfiguracionParar(); //detiene el carro un momento
          estado = GIRE_IZQUIERDA;
        }
        break; 
      }
      
      case GIRE_IZQUIERDA:  { 
        giroTerminado= Giro(-90);
        if   (giroTerminado) {
          ConfiguracionParar(); //detiene el carro un momento
          estado = GIRE_DERECHA;
        }
        break; 
      }
      
      case ESCOGER_DIRECCION: {
        RevisaObstaculoPeriferiaMemoria(); //Revisa los obstáculos presentes en la pose actual
        AsignarDireccionRWM(); //Asigna un ángulo de giro en base al algoritmo Random Walk con Dirección
        resetVarGiro(); //resetea las variables globales utilizadas para medir el giro real
        
        //Para evitar estado de giro permanente
        if(estado!=GIRO){ //Si no se encontraba en giro almacenar el tiempoGiro
          tiempoGiro=millis(); //Almacena el tiempo cuando se cambio a giro para comparar con el limite
        }
        estado= GIRO;
        break;
      }

      case GIRO: {
        giroTerminado=Giro((float)anguloGiro);
        //giroTerminado=Giro(90);

        if ((millis() - tiempoGiro) > limiteGiro){ //Si pasa mas del limite de tiempo tratando de girar se detiene y lo trata como si hubiera completado el giro, evita que se quede intentando girar si está bloqueado
          giroTerminado = true;
        }
        
        if(giroTerminado){
          ConfiguracionParar();
          //serialPrint("Giro real: "); serialPrintln(angActualRobot);
          estado=AVANCE;
        }
        break;
      }

      case CONTROL_POSE: {
        //finPose = AvanzarCoordenada(-300, -1000, errorAvanzarCoordenada); //coordenada x, coordenada y, error permitido, todo en mm
        finPose = AvanzarCoordenada(posicionXAvanzarCoordenada, posicionYAvanzarCoordenada, errorAvanzarCoordenada); //coordenada x, coordenada y, error permitido, todo en mm
        if (finPose){
          CrearObstaculo(15, 0, 0);
          ConfiguracionParar();
          banderaParar = true;
        }
        break;
      }

      case NADA: {
        break;
      }
    }
  }
}



void leerMsgSTM ()  {
  //Función llamada cuando se recibe algo en el puerto I2C conectado al STM32
  //Almacena en una matriz las variables de tipo de sensor, distancia y ángulo
  int cont = 1;
  int tipoSensor = 0;
  int distancia = 0;
  int angulo = 0;
  String acumulado = "";

  Wire.requestFrom(dirSTM, cantBytesMsg);    // Pide cierta cantidad de datos al esclavo
  
  while (0 < Wire.available()) { // ciclo mientras se reciben todos los datos
    char c = Wire.read(); // se recibe un byte a la vez y se maneja como char
    if (c == ',') { //los datos vienen separados por coma
      if (cont == 1) {
        tipoSensor = acumulado.toInt();
        acumulado = "";
      }
      if (cont == 2) {
        distancia = acumulado.toInt();
        acumulado = "";
      }
      cont++;
    } else if (c == '.') { //el ultimo dato viene con punto al final
      angulo = acumulado.toInt();
      acumulado = "";
      cont = 1;
    } else {
      acumulado += c;  //añade el caracter a la cadena anterior
    }
  }
  
  if (tipoSensor>0){ //Si el sensor sigue en 0 significa que el STM no envió nada
    CrearObstaculo(tipoSensor, distancia, angulo);
    resetObstaculos=false;
    if (tipoSensor == 5){ // Si recibe una señal de temperatura detiene el robot en el lugar
      banderaParar = true;
    }
  }

  else if(tipoSensor==-1){ //Si llega tipoSensor==-1 significa que el interruptor está encendido, se va a usar para pedir calibración
    calibrar();
  }

}

void CrearObstaculo(int tipoSensor, int distancia, int angulo){ 
//Agrega un nuevo onstaculo a la lista, usa la poseActual global cuando se llama
  if (ultimoObstaculo == longitudArregloObstaculos - 1) { //si llega al final del arreglo regresa al inicio, sino suma 1
    ultimoObstaculo = 0;
  } else {
    ultimoObstaculo++;
  }
  //Guarda la pose actual donde se detectó el obstáculo y el tipo y detalles del obstáculo
  
  datosSensores[ultimoObstaculo][0] = int(poseActual[0]); //Pose X
  datosSensores[ultimoObstaculo][1] = int(poseActual[1]); //Pose Y
  datosSensores[ultimoObstaculo][2] = int(poseActual[2]); //Orientacion respecto al inicio

  datosSensores[ultimoObstaculo][3] = tipoSensor;
  datosSensores[ultimoObstaculo][4] = distancia;
  datosSensores[ultimoObstaculo][5] = angulo;

  nuevoObstaculo = true;
}


void RecorrerObstaculos(){ 
//Recorre la lista de obstaculos empezando desde el ultimo recibido, permite enviar mensajes anteriores cuando el robot no está detectando nada
  if(!mensajeCreado){ //Verifica que se envio el ultimo mensaje creado para no recorrer la lista sin necesidad
    if(nuevoObstaculo){ //Si se recibio un nuevo obstaculo reiniciar el recorrido de la lista desde ultimoObstaculo
      nuevoObstaculo = false;
      minEnviado = ultimoObstaculo;
    }
    
    else{
      if(minEnviado>=0){
        minEnviado = minEnviado -1;
      }
    }
    
    //CrearMensaje(datosSensores[minEnviado][0], datosSensores[minEnviado][1], datosSensores[minEnviado][2], datosSensores[minEnviado][3], datosSensores[minEnviado][4], datosSensores[minEnviado][5]); //Enviar lista de últimos obstaculos
    CrearMensaje(datosSensores[ultimoObstaculo][0], datosSensores[ultimoObstaculo][1], datosSensores[ultimoObstaculo][2], datosSensores[ultimoObstaculo][3], datosSensores[ultimoObstaculo][4], datosSensores[ultimoObstaculo][5]); //Enviar solo ultimo obstaculo para debugging
   }
}

/// @brief Actualiza la ubicación del robot
void ActualizarUbicacionReal() {
  //Función que actualiza la ubicación actual en base al avance anterior y la orientación actual dada por el magnetometro

  orientacion = (float)(ultimoAngMagnet - magInicioOrigen); //Orientacion del robot
  if(orientacion > 180){ //Correción para tener valores entre -180 y 180
    orientacion = orientacion - 360;
  }
  else if(orientacion < -180){
    orientacion = orientacion + 360;
  }

  poseActual[2] = orientacion;
  
  orientacion = PI * orientacion / 180; //Conversion a radianes para trigonometria
  
  distanciaAvanzada = (int)calculaDistanciaLinealRecorrida();
  if(distanciaAvanzada >= limiteObstaculos){
    resetObstaculos = true;
    obstaculoIzquierda=false;
    obstaculoAdelante=false;
    obstaculoDerecha=false;
  }

  //Calcular nueva posición basado en la distancia y el angulo en que se movio (convertir coordenadas polares a rectangulares)
  poseActual[0] = (int)(poseActual[0] + (distanciaAvanzada * cos(orientacion))); //coordenada X
  poseActual[1] = (int)(poseActual[1] + (distanciaAvanzada * sin(orientacion))); //coordenada Y
}

/// @brief Interrupción que realiza el STM32
void DeteccionObstaculo(){
  //Función tipo interrupción llamada cuando se activa el pin de detección de obstáculo del STM32

  // Primero verifica que no esté en el proceso de arranque
  if (!datoCoordenadas){ // Recibe la distancia del sharp que hay entre el robot y el frente 
    distCoordenadas = leerDistanciaSTM();
    //serialPrintln(distCoordenadas);
    datoCoordenadas = true;
  }else{// Modo normal de operación
    //Son obstáculos que requieren que el robot retroceda y cambie de dirección inmediatamente
    if(estado!=RETROCEDA && giroTerminado==1 && tiempoActual>2000){
      //Solo si el estado ya no es retroceder.  También que no está haciendo un giro 
      //y para que ignore las interrupciones los primeros segundos al encenderlo
      estado=RETROCEDA;
      ConfiguracionParar(); //Se detiene un momento y reset de encoders 
      tiempoRetroceso= tiempoActual; //Almacena el ultimo tiempo para usarlo en el temporizador
    }
  }
}


//****Funciones para el algortimo RWD*****

void RevisaObstaculoPeriferia() {
  //Función que revisa los obstáculos detectados previamente y determina cuales están en la periferia actual
  //Retorna una simplificación de si hay ostáculo adelante, a la derecha o atrás

  for (int i = 0; i <= ultimoObstaculo; i++) {
    //Busca si  la pose actual calza con la pose cuando se detectó el obstáculo. Uso un margen de tolerancia para la detección
    if ( (abs(poseActual[0] - datosSensores[i][0]) < unidadAvance) &&
         (abs(poseActual[1] - datosSensores[i][1]) < unidadAvance) &&
         datosSensores[i][2] == poseActual[2]) {

      //Evalua el ángulo del obstáculo y lo simplifica a si hay obstáculo adelante, a la derecha o a la izquierda
      if (datosSensores[i][5] >= -45 && datosSensores[i][5] <= 45) {
        obstaculoAdelante = true;
      }
      if (datosSensores[i][5] > 55 && datosSensores[i][5] < 95) {
        obstaculoDerecha = true;
      }
      if (datosSensores[i][5] > -95 && datosSensores[i][5] < -55) {
        obstaculoIzquierda = true;
      }     
    }
  }
}

void RevisaObstaculoPeriferiaMemoria() {
  //Función que revisa los obstáculos detectados previamente y determina cuales están en la periferia actual
  //Retorna una simplificación de si hay obstáculo adelante, a la derecha, a la izquierda o atrás

  for (int i = 0; i <= ultimoObstaculo; i++) {  //Revisa todos los obstaculos que se han detectado y que siguen en memoria
    //Busca si  la pose actual calza con la pose cuando se detectó el obstáculo. Uso un margen de tolerancia para la detección
    if ( (abs(poseActual[0] - datosSensores[i][0]) < limiteObstaculos) &&
         (abs(poseActual[1] - datosSensores[i][1]) < limiteObstaculos)){

      if(datosSensores[i][3] > 0 && datosSensores[i][3] <=4){ //Verifica si hay un obstaculo o si termino correctamente el avance, los "obstaculos" 5 y 6 son de temperatura y batería baja, se ignoran para el movimiento
        sinObstaculo= false;
        
        int anguloAbsoluto = datosSensores[i][5] + datosSensores[i][2]; //Cambiar angulo de obstaculo para ser respecto al origen de coordenadas
        //Correción para tener valores entre -180 y 180
        if(anguloAbsoluto > 180){anguloAbsoluto = anguloAbsoluto - 360;}
        else if(anguloAbsoluto < -180){anguloAbsoluto = anguloAbsoluto + 360;}

        int anguloReal = anguloAbsoluto - poseActual[2]; //Cambiar angulo para ser respecto a la orientacion actual del robot
        //Correción para tener valores entre -180 y 180
        if(anguloReal > 180){anguloReal = anguloReal - 360;}
        else if(anguloReal < -180){anguloReal = anguloReal + 360;}


        //Evalua el ángulo del obstáculo y lo simplifica a si hay obstáculo adelante, a la derecha o a la izquierda
        if (anguloReal >= -45 && anguloReal <= 45) {
          obstaculoAdelante = true;
          //Serial.println("Obstaculo adelante");
        }
        else if (anguloReal > 45 && anguloReal <= 135) {
          obstaculoDerecha = true;
          //Serial.println("Obstaculo derecha");
        }
        else if (anguloReal >= -135 && anguloReal < -45) {
          obstaculoIzquierda = true;
          //Serial.println("Obstaculo izquierda");
        }
        else if (anguloReal < -135 || anguloReal > 135) {
          obstaculoAtras = true;
          //Serial.println("Obstaculo atras");
        }
      }
      else{
        sinObstaculo= true;
      }
    }
  }
}

void AsignarDireccionRWD() {
  //Función que escoge la dirección de giro para el robot en base al algoritmo Random Walk con Dirección
  //Actualiza la variable global anguloGiro

  int diferenciaOrientacion = poseActual[2] - (int)direccionGlobal;
  int minRandom = 0;
  int maxRandom = 3;
  int incrementoPosible = 90;

   bool adelante=false;
  bool izquierda=false;
  bool derecha=false;
  bool atras=false;

  if (-45.0<=poseActual[2]<=45.0){adelante=true;}
  else if (45<poseActual[2]<=135.0){derecha=true;}
  else if (-135.0<=poseActual[2]<-45.0){izquierda=true;}
  else {atras=true;}

  //En esta condición especial se asigna una nueva dirección global
  if (obstaculoAdelante && adelante) {
    direccionGlobal = (orientacionesRobot)(random(-1, 3) * 90);
  }
  //Si hay un obstáculo o se está en una orientación diferente a la global, se restringen las opciones del aleatorio
  if (obstaculoIzquierda || izquierda) {
    minRandom++;
  }
  if (obstaculoDerecha || derecha) {
    maxRandom--;
  }
  if (obstaculoAdelante || atras) {
    incrementoPosible = 180; //Un obstáculo adelante es un caso especial que implica girar a la derecha o izquierda (diferencia de 180°)
    maxRandom--;
  }
  if (obstaculoIzquierda && obstaculoAdelante && obstaculoDerecha) { //condición especial (callejón)
    anguloGiro = 180;
    direccionGlobal = (orientacionesRobot)(random(-1, 3) * 90);
    //Serial.print("5");
  } 
  else {
    //La ecuación del ángulo de giro toma en cuenta todas las restricciones anteriores y lo pasa a escala de grados (nomenclatura de orientación)
    anguloGiro = random(minRandom, maxRandom) * incrementoPosible - 90;
  }

  //Reset de la simplificación sobre obstáculo en la pose actual
  obstaculoAdelante = false;
  obstaculoDerecha = false;
  obstaculoIzquierda = false;
}

void AsignarDireccionRWM(){ //Nuevo random walk con memoria para mejorar comportamiento
//Función que escoge la dirección de giro para el robot en base al algoritmo Random Walk con Dirección
  //Actualiza la variable global anguloGiro

  int direccionesPosibles[4]={0,-90,90,180}; //Angulos posibles que va a pedir a Giro(), positivo es contra reloj (izquierda) por el momento 6-13-23
  int direccionGlobalAnterior=direccionGlobal;

  int orientacionActual;
  bool adelante=false;
  bool izquierda=false;
  bool derecha=false;
  bool atras=false;

  //Aproximar orientación real a cuadrantes para calculo
  if (-45.0<=poseActual[2] && poseActual[2]<=45.0){orientacionActual=0;} //Adelante
  else if (45<poseActual[2] && poseActual[2]<=135.0){orientacionActual=90;} //Derecha
  else if (-135.0<=poseActual[2] && poseActual[2]<-45.0){orientacionActual=-90;} //Izquierda
  else {orientacionActual=180;} //Atras

  orientacionActual=orientacionActual - direccionGlobal; //Ver diferencia de orientación real con la deseada
  if (abs(orientacionActual)>=270){orientacionActual=orientacionActual/-3;} //Si da angulo de 270 corregir para evitar que pida dar una vuelta completa
  //Valores respecto a variable orientacionActual toman en cuenta la direccionGlobal
  if (-45.0<=orientacionActual && orientacionActual<=45.0){adelante=true;}
  else if (45<orientacionActual && orientacionActual<=135.0){derecha=true;}
  else if (-135.0<=orientacionActual && orientacionActual<-45.0){izquierda=true;}
  else {atras=true;}

  if(sinObstaculo){ //Si no hay obstaculos decidir dirección con direccion global
    if(adelante){
      anguloGiro = direccionesPosibles[random(0, 3)];
      return;
    }
    else if(derecha){
      if(random(0,2)){
        anguloGiro = direccionesPosibles[0];
        return;
      }
      else{
        anguloGiro = direccionesPosibles[1];
        return;
      }
    }
    else if(izquierda){
      if(random(0,2)){
        anguloGiro = direccionesPosibles[0];
        return;
      }
      else{
        anguloGiro = direccionesPosibles[2];
        return;
      }
    }
    else if(atras){
      if(random(0,2)){
        anguloGiro = direccionesPosibles[2];
        return;
      }
      else{
        anguloGiro = direccionesPosibles[1];
        return;
      }
    }
  }

//Obstaculos se ven respecto a la posición actual del robot
bool anguloSeleccionado=false;
while (!anguloSeleccionado)
{
  if(obstaculoAdelante && obstaculoIzquierda && obstaculoDerecha){ //Revisar condición de callejon
    anguloGiro = direccionesPosibles[3];
    anguloSeleccionado = true;
  }
  switch(random(0,3)){ //Seleccionar una dirección en la que ir y revisar si es posible basado en los obstaculos presentes
    case 0://Ir a izquierda
      if(!obstaculoIzquierda){//Si no hay obstaculo a la izquierda aceptar decision
        anguloGiro = direccionesPosibles[1];
        anguloSeleccionado = true;
      }
      break;

    case 1://Ir adelante
      if(!obstaculoAdelante){
        anguloGiro = direccionesPosibles[0];
        anguloSeleccionado = true;
      }
      break;

    case 2://Ir a la derecha
      if(!obstaculoDerecha){
        anguloGiro = direccionesPosibles[2];
        anguloSeleccionado = true;
      }
      break;

    default:
    break;
  }
}

  //En esta condición especial se asigna una nueva dirección global, no debería tener que entrar a este ciclo, revisar si logra entrar
  if (obstaculoAdelante && adelante) { //Debe elegir nueva dirección global aleatoria diferente a la actual
    direccionGlobalAnterior=direccionGlobal;
    while(direccionGlobalAnterior==direccionGlobal){
      direccionGlobal = (orientacionesRobot)(random(-1, 3) * 90);
    }
  }

  //Reset de la simplificación sobre obstáculo en la pose actual
  obstaculoAdelante = false;
  obstaculoDerecha = false;
  obstaculoIzquierda = false;
  sinObstaculo = false;
}


//***Funciones para control de giro del robot****

float ConvDistAngular(float cantPulsos) {
  //Calcula la distancia angular (en grados) que se ha desplazado el robot.
  //Para robots diferenciales, en base a un giro sobre su eje (no arcos)
  //Recibe la cantidad de pulsos de la rueda. Devuelve un ángulo en grados.

  float despAngular = cantPulsos / (distanciaCentroARueda * pulsosPorMilimetro) * 180 / PI;
  return despAngular;
}

int ControlPosGiroRueda( float posRef, float posActual, float& sumErrorGiro, float& errorAnteriorGiro ) {
  //Funcion para implementar el control PID por posición en una rueda.
  //Se debe tener un muestreo en tiempos constantes, no aparecen las constantes de tiempo en la ecuación, sino que se integran con las constantes Ki y Kd
  //Se recibe la posición de referencia, la posición actual (medida con los encoders), el error acumulado (por referencia), y el valor anterior del error (por referencia)
  //Entrega el valor de ciclo de trabajo (PWM) que se enviará al motor CD

  float errorGiro = posRef - posActual; //se actualiza el error actual

  //se actualiza el error integral y se restringe en un rango, para no aumentar sin control
  //como son variables que se mantienen en ciclos futuros, se usan variables globales
  sumErrorGiro += errorGiro * tiempoMuestreoS;
  sumErrorGiro = constrain(sumErrorGiro, errorMinIntegralGiro, errorMaxIntegralGiro);

  //error derivativo (diferencial)
  float difErrorGiro = (errorGiro - errorAnteriorGiro) / tiempoMuestreoS;

  //ecuación de control PID
  float pidTermGiro = (KpGiro * errorGiro) + (KiGiro * sumErrorGiro) + (KdGiro * difErrorGiro);

  //Se limita los valores máximos y mínimos de la acción de control, para no saturarla
  pidTermGiro = constrain( int (pidTermGiro), limiteInferiorCicloTrabajoGiro, limiteSuperiorCicloTrabajoGiro);

  //Restricción para manejar la zona muerta del PWM sobre la velocidad
  if (-cicloTrabajoMinimo < pidTermGiro && pidTermGiro < cicloTrabajoMinimo) {
    pidTermGiro = 0;
  }

  //actualiza el valor del error para el siguiente ciclo
  errorAnteriorGiro = errorGiro;

  return  pidTermGiro;
}

bool EstadoEstacionario (int pwmRuedaDer, int pwmRuedaIzq, int& contCiclosEstacionario) {
  //Función que revisa el estado de la acción de control y si se mantiene varios ciclos en cero, asume que ya está en el estado estacionario
  //Recibe los ciclos de trabajo en cada rueda y una variable por referencia donde se almacenan cuantos ciclos seguidos se llevan
  //Devuelve una variable que es TRUE si ya se alcanzó el estado estacionario

  bool estadoEstacionarioAlcanzado = false;

  if (pwmRuedaDer == 0 && pwmRuedaIzq == 0) {
    contCiclosEstacionario++;
    if (contCiclosEstacionario > minCiclosEstacionario) {
      //ResetContadoresEncoders();
      contCiclosEstacionario = 0;
      estadoEstacionarioAlcanzado = true;
    }
  } else {
    contCiclosEstacionario = 0;
  }

  return estadoEstacionarioAlcanzado;
}

bool Giro(float grados) {
//Función que ejecuta el giro del robot sobre su propio eje. Usa control PID en cada rueda
//Devuelve true cuando se alcanza la posición angular deseada

  float factorFus=0.0;  //Peso de medicion de magnetometro y MPU en el calculo del giro    0.7 anteriormente
  angActualRobot=GiroReal(anguloMPU,anguloInicial,ultimoAngMagnet); //calcula el giro real
  //angActualRobot=GiroReal(anguloMPU,0,anguloEncoder);

  posActualRuedaDerecha = ConvDistAngular(contPulsosDerecha)*(1-factorFus)+factorFus*angActualRobot;
  int cicloTrabajoRuedaDerecha = ControlPosGiroRueda( -grados, posActualRuedaDerecha, sumErrorGiroDer, errorAnteriorGiroDer );

  posActualRuedaIzquierda = ConvDistAngular(contPulsosIzquierda)*(1-factorFus)+factorFus*angActualRobot;
  int cicloTrabajoRuedaIzquierda = ControlPosGiroRueda( grados, -posActualRuedaIzquierda, sumErrorGiroIzq, errorAnteriorGiroIzq);

  ConfiguraEscribePuenteH (cicloTrabajoRuedaDerecha, cicloTrabajoRuedaIzquierda);

  bool giroListo = EstadoEstacionario (cicloTrabajoRuedaDerecha, cicloTrabajoRuedaIzquierda, contCiclosEstacionarioGiro);
  if (giroListo){
    sumErrorGiroDer = 0;
    errorAnteriorGiroDer = 0;
    sumErrorGiroIzq = 0;
    errorAnteriorGiroIzq = 0;
  }

  return giroListo;

}

bool correccionGiro(){ //Corrige la orientacion del robot para que quede en los cuadrantes    NO FUNCIONAL
  Serial.println("Correccion");

  //Aproximar orientación real a cuadrantes para calculo
  if (-45.0<=poseActual[2] && poseActual[2]<=45.0){//Adelante
    anguloGiro=(-poseActual[2]);
  } 
  
  else if (45<poseActual[2] && poseActual[2]<=135.0){//Derecha
    anguloGiro=(90-poseActual[2]);
  } 
  
  else if (-135.0<=poseActual[2] && poseActual[2]<-45.0){//Izquierda
    anguloGiro=(-90-poseActual[2]);
  }
  
  else {//Atras  
    if(poseActual[2]>0){
      anguloGiro=(180-poseActual[2]);
    }
    else{
      anguloGiro=(-180-poseActual[2]);
    }
  }

  anguloGiro=-anguloGiro;

  if(abs(anguloGiro)<5){
    return true;
  }
  else{
    Serial.println(anguloGiro);
    return false;
  }
}

//***Funciones para configuración del Puente H y por ende la dirección del robot

void ConfiguracionParar() {
  //Deshabilita las entradas del puente H, por lo que el carro se detiene
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, LOW);
  ResetContadoresEncoders();
  resetAvance();
  delay(250);
}

void ConfiguracionAvanzar() {
  //Permite el avance del carro al poner las patillas correspondientes del puente H en alto
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, HIGH);
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, HIGH);
}

void ConfiguracionRetroceder() {
  //Configura el carro para retroceder al poner las patillas correspondientes del puente H en alto
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, LOW);
}

void ConfiguracionGiroDerecho() {
  //Configura las patillas del puente H para realizar un giro derecho
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, HIGH);
}

void ConfiguracionGiroIzquierdo() {
  //Configura las patillas del puente H para realizar un giro izquierdo
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, HIGH);
  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, LOW);
}

void ConfiguraEscribePuenteH (int pwmRuedaDer, int pwmRuedaIzq) {
  //Determina si es giro, avance, o retroceso en base a los valores de PWM y configura los pines del Puente H
  if (pwmRuedaDer >= 0 && pwmRuedaIzq >= 0) {
    ConfiguracionAvanzar();
    analogWrite(PWMB, pwmRuedaDer);
    analogWrite(PWMA, pwmRuedaIzq);

  } else if (pwmRuedaDer < 0 && pwmRuedaIzq < 0) {
    ConfiguracionRetroceder();
    analogWrite(PWMB, -pwmRuedaDer);
    analogWrite(PWMA, -pwmRuedaIzq);

  } else if (pwmRuedaDer > 0 && pwmRuedaIzq < 0) {
    ConfiguracionGiroDerecho();
    analogWrite(PWMB, pwmRuedaDer);
    analogWrite(PWMA, -pwmRuedaIzq);

  } else if (pwmRuedaDer < 0 && pwmRuedaIzq > 0) {
    ConfiguracionGiroIzquierdo();
    analogWrite(PWMB, -pwmRuedaDer);
    analogWrite(PWMA, pwmRuedaIzq);
  }
}


//***Funciones para manejo de las señales de los encoders***

void ResetContadoresEncoders() {
  //Realiza una inicialización de las variables que cuentan cantidad de pulsos y desplazamiento de la rueda

  contPulsosDerecha = 0;
  contPulsosIzquierda = 0;

  posActualRuedaDerecha = 0.0;
  posActualRuedaIzquierda = 0.0;

  contPulsosDerPasado = 0;
  contPulsosIzqPasado = 0;
}

void PulsosRuedaDerechaC2_2() {
  //Manejo de interrupción del canal C2 del encoder de la rueda derecha con lógica binaria y tabla
  estadoEncoderDer = estadoEncoderDer << 2; //Mover valor anterior de encoder a la izquierda

  //lectura de los encoders en puertos directamente, devuelve TRUE si está en alto
  bool C1 = PORT->Group[PORTA].IN.reg & PORT_PA02;
  bool C2 = PORT->Group[PORTB].IN.reg & PORT_PB08;

  estadoEncoderDer = ((estadoEncoderDer | (0b10*C1) + (0b1*C2)) & 0b1111); //Colocar nuevo valor de encoder en primeros lugares y eliminar todo lo demás
  //El resultado da estadoEncoder = estadoEncoderAnterior en bit 3 y 2, estadoEncoderActual en bit 1 y 0

  contPulsosDerecha = contPulsosDerecha + tablaEncoder[estadoEncoderDer]; //Buscar en tabla que hacer al contador de pulsos
}

void PulsosRuedaIzquierdaC2_2() {
  //Manejo de interrupción del canal C2 del encoder de la rueda izquierda con lógica binaria y tabla
  estadoEncoderIzq = estadoEncoderIzq << 2; //Mover valor anterior de encoder a la izquierda

  //lectura de los encoders en puertos directamente, devuelve TRUE si está en alto
  bool C1 = PORT->Group[PORTB].IN.reg & PORT_PB09;
  bool C2 = PORT->Group[PORTA].IN.reg & PORT_PA04;

  estadoEncoderIzq = ((estadoEncoderIzq | (0b10*C1) + (0b1*C2)) & 0b1111); //Colocar nuevo valor de encoder en primeros lugares y eliminar todo lo demás
  //El resultado da estadoEncoder = estadoEncoderAnterior en bit 3 y 2, estadoEncoderActual en bit 1 y 0

  contPulsosIzquierda = contPulsosIzquierda + tablaEncoder[estadoEncoderIzq]; //Buscar en tabla que hacer al contador de pulsos
}


///****Funciones de control de velocidad del robot*****

float calculaVelocidadRueda(int& contPulsos, int& contPulsosPasado) {
  //Función que calcula la velocidad de una rueda en base a la cantidad de pulsos del encoder y el tiempo de muestreo

  float velActual = ((contPulsos - contPulsosPasado) / pulsosPorMilimetro) / ((float)tiempoMuestreo / conversionMicroSaSDiv); //velocidad en mm por s
  contPulsosPasado = contPulsos;

  return velActual;
}

float calculaDistanciaLinealRecorrida() {
  //Función que realiza el cálculo de la distancia lineal recorrida por cada rueda
  //Devuelve el promedio de las distancias

  distLinealRuedaDerecha = (float)contPulsosDerecha / pulsosPorMilimetro;
  distLinealRuedaIzquierda = - (float)contPulsosIzquierda / pulsosPorMilimetro;
  float DistanciaLineal = (distLinealRuedaDerecha + distLinealRuedaIzquierda) / 2.0;

  return DistanciaLineal;
}

int ControlVelocidadRueda( float velRef, float velActual, float& sumErrorVel, float& errorAnteriorVel ) {
  //Funcion para implementar el control PID por velocidad en una rueda.
  //Se debe tener un muestreo en tiempos constantes, no aparecen las constantes de tiempo en la ecuación, sino que se integran con las constantes Ki y Kd
  //Se recibe la posición de referencia, la posición actual (medida con los encoders), el error acumulado (por referencia), y el valor anterior del error (por referencia)
  //Entrega el valor de ciclo de trabajo (PWM) que se enviará al motor CD

  float errorVel = velRef - velActual; //se actualiza el error actual

  //se actualiza el error integral y se restringe en un rango, para no aumentar sin control
  //como son variables que se mantienen en ciclos futuros, se usan variables globales
  sumErrorVel += errorVel * tiempoMuestreoS; //Anteriormente se comportaba como producto de sumatoria cuando debía ser sumatoria de productos
  sumErrorVel = constrain(sumErrorVel, errorMinIntegralVelocidad, errorMaxIntegralVelocidad);

  //error derivativo (diferencial)
  float difErrorVel = (errorVel - errorAnteriorVel) / tiempoMuestreoS / 10; //Se divide el factor derivativo entre 10 debido a la inestabilidad de la velocidad por falta de precisión en la medición

  //ecuación de control PID
  float pidTermVel = (KpVel * errorVel) + (KiVel * sumErrorVel) + (KdVel * difErrorVel);  //Agregada división entre 5 porque PID saturaba, buscar donde está el error

  //Se limita los valores máximos y mínimos de la acción de control, para no saturarla
  pidTermVel = constrain( (int)pidTermVel, limiteInferiorCicloTrabajoVelocidad, limiteSuperiorCicloTrabajoVelocidad);

  //  //Restricción para manejar la zona muerta del PWM sobre la velocidad
  //  if (-cicloTrabajoMinimo < pidTermVel && pidTermVel < cicloTrabajoMinimo){
  //    pidTermVel = 0;
  //  }

  //actualiza el valor del error para el siguiente ciclo
  errorAnteriorVel = errorVel;

  return  ((int)pidTermVel);
}


bool AvanzarDistancia(int distanciaDeseada) {
  //Avanza hacia adelante una distancia definida en mm a velocidad constante sincronizando la velocidad de ambas ruedas para asegurar movimiento en línea recta
  //Devuelve true cuando alcanzó la distancia deseada

  float velSetPoint= velRequerida;
  if (distanciaDeseada<0){  //Si la distancia deseada es negativa, significa retroceder y por ende velocidad negativa (y un poco más despacio)
    velSetPoint= -1 * velRequerida * 0.75;
  }

  velActualDerecha= calculaVelocidadRueda(contPulsosDerecha, contPulsosDerPasado);
  velActualIzquierda= -1.0 * calculaVelocidadRueda(contPulsosIzquierda, contPulsosIzqPasado); //como las ruedas están en espejo, la vel es negativa cuando avanza, por eso se invierte
  
  float errorVelocidad = velActualDerecha - velActualIzquierda; //Error entre la velocidad de las dos ruedas
  float velSetPointDerecha = velSetPoint - errorVelocidad * correcionVelRuedas;
  float velSetPointIzquierda = velSetPoint + errorVelocidad * 0.0;

  int cicloTrabajoRuedaDerecha = ControlVelocidadRueda(velSetPointDerecha, velActualDerecha, sumErrorVelDer, errorAnteriorVelDer);
  int cicloTrabajoRuedaIzquierda = ControlVelocidadRueda(velSetPointIzquierda, velActualIzquierda, sumErrorVelIzq, errorAnteriorVelIzq);
   
  ConfiguraEscribePuenteH (cicloTrabajoRuedaDerecha, cicloTrabajoRuedaIzquierda);

  float distanciaAvanzada= calculaDistanciaLinealRecorrida();

  bool avanceListo = false; 
  if (abs(distanciaAvanzada) >= abs(distanciaDeseada)) {
    sumErrorVelDer=0;
    errorAnteriorVelDer=0;
    sumErrorVelIzq=0;
    errorAnteriorVelIzq=0;
    avanceListo = true; 
    //ResetContadoresEncoders();
  }

  return avanceListo;
}

void AvanzarIndefinido() {
  //Avanza hacia adelante indefinidamente
  //Luego de utilizarse, se debe llamar a ResetContadoresEncoders()

  velActualDerecha = calculaVelocidadRueda(contPulsosDerecha, contPulsosDerPasado);
  int cicloTrabajoRuedaDerecha = ControlVelocidadRueda(velRequerida, velActualDerecha, sumErrorVelDer, errorAnteriorVelDer);

  velActualIzquierda = -1.0 * calculaVelocidadRueda(contPulsosIzquierda, contPulsosIzqPasado); //como las ruedas están en espejo, la vel es negativa cuando avanza, por eso se invierte
  int cicloTrabajoRuedaIzquierda = ControlVelocidadRueda(velRequerida, velActualIzquierda, sumErrorVelIzq, errorAnteriorVelIzq);

  ConfiguraEscribePuenteH (cicloTrabajoRuedaDerecha, cicloTrabajoRuedaIzquierda);

}

/// @brief Funcion que permite al Atta ir hacia una coordenada en especifico
/// @param coordenadaXDeseada Posicion X de la posicion deseada
/// @param coordenadaYDeseada Posicion Y de la posicion deseada
/// @param errorPermitido Error permitido de la posicion XY del movimiento
/// @return true cuando se completa el movimiento y false cuando se esta ejecutando el movimiento
bool AvanzarCoordenada(int coordenadaXDeseada, int coordenadaYDeseada, float errorPermitido) {
  
  static float tiempoPrevioCoordenada = 0; // Actualmente no se usa, ganancia integral y derivativa se deben modificar
  static float tiempoCoordenada = micros(); // Actualmente no se usa, ganancia integral y derivativa se deben modificar
  static bool finMovimiento = false; // Inicializa la funcion
  static float controlIntegral = 0; // Variable que almacena el valor integral, ahorita no es funcional
  static float velocidadAngular = 0; // Velocidad angular del atta

  if (tiempoPrevioCoordenada == 0){
    tiempoPrevioCoordenada = tiempoMaquinaEstados;
  }
  
  //Desplazamiento hasta una coordenada deseada
  //Devuelve true cuando alcanzó la coordenada deseada

  float errorCoordenadaX = coordenadaXDeseada - poseActual[0];
  float errorCoordenadaY = coordenadaYDeseada - poseActual[1];

  errorOrientacion = atan2(errorCoordenadaX,errorCoordenadaY) - anguloOrientacion*(PI/180); //Respuesta en radianes

  finMovimiento = false;
  float errorPose = sqrt(pow(errorCoordenadaX,2)+pow(errorCoordenadaY,2));
  //Serial.print("errorPose: ");
  //Serial.println(errorPose);
  if (errorPose <= errorPermitido) {
    finMovimiento = true;
    banderaParar = true;
    controlIntegral = 0;
    return finMovimiento;
  }
  
  velActualDerecha = calculaVelocidadRueda(contPulsosDerecha, contPulsosDerPasado);
  velActualIzquierda = -1.0 * calculaVelocidadRueda(contPulsosIzquierda, contPulsosIzqPasado); //como las ruedas están en espejo, la vel es negativa cuando avanza, por eso se invierte

  // Control proporcional del carro
  float controlProporcional = KpPose * errorOrientacion;
  
  // Contorl derivativo del carro
  tiempoCoordenada = micros();
  double dt = (tiempoCoordenada - tiempoPrevioCoordenada)/1000000.0;
  float controlDerivativo = KdPose * (errorOrientacion - errorAnt_1_Orientacion)/dt;

  // Control integral del carro
  controlIntegral += KiPose * (errorOrientacion + 2*errorAnt_1_Orientacion + errorAnt_2_Orientacion)/2;
  tiempoPrevioCoordenada = tiempoCoordenada;

  float controlPose = controlProporcional;// + controlDerivativo + controlIntegral; // Las constantes derivativas e integrales estan en cero

  velocidadAngular = (velActualDerecha - velActualIzquierda)/(2*distanciaCentroARueda) + controlPose;

  float nuevaVelocidadRuedaDerecha = velBase - velocidadAngular*(distanciaCentroARueda);
  float nuevaVelocidadRuedaIzquierda = velBase + velocidadAngular*(distanciaCentroARueda);

  //serialPrint("e ");serialPrint(errorOrientacion);serialPrint(" VL ");serialPrint(nuevaVelocidadRuedaIzquierda);serialPrint(" VR ");serialPrintln(nuevaVelocidadRuedaDerecha);

  int cicloTrabajoRuedaDerecha = ControlVelocidadRueda(nuevaVelocidadRuedaDerecha, velActualDerecha, sumErrorVelDer, errorAnteriorVelDer);
  int cicloTrabajoRuedaIzquierda = ControlVelocidadRueda(nuevaVelocidadRuedaIzquierda, velActualIzquierda, sumErrorVelIzq, errorAnteriorVelIzq);
  
  ConfiguraEscribePuenteH (cicloTrabajoRuedaDerecha, cicloTrabajoRuedaIzquierda);

  distLinealRuedaDerechaAnterior = distLinealRuedaDerecha;
  distLinealRuedaIzquierdaAnterior = distLinealRuedaIzquierda;
  errorAnt_2_Orientacion = errorAnt_1_Orientacion;
  errorAnt_1_Orientacion = errorOrientacion;
  
  return finMovimiento;
}

//*****Funciones para manejar la orientación*********

void resetVarGiro() {
  //Funcion que resetea las variables globales de la función GiroReal
  //cuadranteAnterior = NULL;
  //cuadranteActual = NULL;  No utilizado
  cambio12=false;
  cambio21=false;
  complemento=false; 
  anguloInicial= ultimoAngMagnet; //guardar el angulo antes de comenzar el giro
  cuadranteAnterior=buscaCuadrante(anguloInicial);
  resetVarMPU();
}

int buscaCuadrante(float angulo) {
  //Determina el cuadrante donde se encuentra el angulo ingresado como parámetro, devuelve
  //un entero del 1 al 4 indicando el cuadrante, esta convención se basa en los cuadrantes trigonométricos normales (I,II,III,IV)
  int cuadrante;
  if (angulo >= 0.0 and angulo <= 90.0) {
    cuadrante = 1;
  }
  else if (angulo > 90.0 and angulo <= 180.0) {
    cuadrante = 4;
  }
  else if (angulo > 180.0 and angulo <= 270.0) {
    cuadrante = 3;
  }
  else if (angulo > 270.0 and angulo <= 360.0) {
    cuadrante = 2;
  }
  return cuadrante;
}


void detectaCambio(float angulo){
//Funcion que detecta los cuadrantes que recorre el magnetometro, básicamente detecta si se debe usar la diferencia entre
//angulo inicial y final del magnetometro o si se usa su complemento, tiene como entrada el angulo del magnetometro

      int cuadranteActual = buscaCuadrante(angulo); 

      //Segun el cambio entre cuadrante 1-2 o 2-1 se sabe si debe usar complemento o la diferencia normal de angulos
      if (cambio12 or cambio21){ 
          if (cuadranteAnterior==1 and cuadranteActual==2){ //cambio de 1 hacia 2
            if (cambio12){complemento=true;} 
            else {complemento=false;}
            }
          else if(cuadranteAnterior==2 and cuadranteActual==1){ //cambio de 2 hacia 1
            if (cambio21){complemento=true;}
            else{complemento=false;}
            }
        }

      //Inicializa las variables cambio12 y cambio21
      else{
      if (cuadranteAnterior==1 and cuadranteActual==2){
          cambio12=true;
          complemento=true;
      }
      else if (cuadranteAnterior==2 and cuadranteActual==1){
            cambio21=true;
            complemento=true;
        }
      }

     //guarda el cuadrante anterior
     cuadranteAnterior = cuadranteActual; 
}

float GiroReal(float angMPU, float angMagInicial, float angMagFinal) {
//Funcion que calcula el giro real como un promedio ponderado entre los valores del giroscopio (yaw)
//y los valores del magnetometro
    float difMag;
    //Realiza el calculo segun el estado de complemento (si fue usado el complemento del ángulo)
    if (complemento){
      difMag=360-abs(angMagFinal-angMagInicial);
      }else{
      difMag=abs(angMagFinal-angMagInicial);
      }
    float facFus= 0.0; //factor de cuanto pesa el valor del MPU (valor de 0 a 1)  ******************************************************************************Cambio de 0,5
    if (angMPU>0){
      return (abs(angMPU)*facFus + difMag*(1-facFus));
    }
    else {
       return -1*(abs(angMPU)*facFus + difMag*(1-facFus));
      }
}




//******Funciones de la EEPROM******

float constrVar(byte LSB, byte MSB, byte dec) {
  //Funcion que "arma" un numero flotante a partir de sus 8 bits LSB, sus 8 bits MSB y 8 bits para la
  //parte decimal del numero, devuelve el numero reconstruido como flotante
  int piv1, piv2;
  float nuevo, decs;
  piv1 = LSB;
  piv2 = MSB << 8;
  nuevo = piv1 | piv2;
  decs = float(dec);
  nuevo = nuevo + decs / 100;
  return nuevo;
}

void transfVar(float num, byte &var1, byte &var2, byte &varDec) {
  int nuevoNum = int(num);
  int desplazamiento;
  float dec;
  desplazamiento = nuevoNum << 8;
  var1 = desplazamiento >> 8; //LSB
  var2 = nuevoNum >> 8; //MSB
  dec = 100 * (num - nuevoNum);
  varDec = int(dec);
}

void eepromEscribe(byte dir, byte dirPag, byte data) {
  segundoI2C.beginTransmission(dir);
  segundoI2C.write(dirPag);
  segundoI2C.write(data);
  segundoI2C.endTransmission();
}

byte eepromLectura(int dir, int dirPag) {
  //Funcion que lee datos de la memoria eeprom, pasa como parametros la direccion del dispositivo (hexadecimal)
  //y la direccion de pagina donde se desea leer, devuelve el dato de 8 bits contenido en esa direccion
  segundoI2C.beginTransmission(dir);
  segundoI2C.write(dirPag);
  segundoI2C.endTransmission();
  segundoI2C.requestFrom(dir, 1);
  if (segundoI2C.available())
    return segundoI2C.read();
  else
    return 0xFF;
}

float leerDatoFloat(int dirPagInicial) {
  //Funcion que lee datos flotantes de la memoria eeprom, pasa como parametro la posicion inicial
  //de memoria para comenzar a leer. Se leen los primeros 8 bits que serian LSB y luego los 8 bits correspondientes al MSB
  //luego se leen los 8 bits correspondientes a la parte decimal y por ultimo 8 bits que determinan el signo (==0 seria positivo e ==1 seria negativo)

  byte signo; //signo del numero, 1 para negativo, 0 para positivo
  byte varr1; //primeros 8 bits para guardar en la EEPROM (LSB)
  byte varr2; //ultimo 8 bits para guardar en la EEPROM (MSB)
  byte varrDec; //parte decimal para guardar en la EEPROM
  float dato;
  //Lee 4 posiciones de memoria a partir de la ingresada como parametro
  varr1 = eepromLectura(dirEEPROM, dirPagInicial);
  varr2 = eepromLectura(dirEEPROM, dirPagInicial + 1);
  varrDec = eepromLectura(dirEEPROM, dirPagInicial + 2);
  signo = eepromLectura(dirEEPROM, dirPagInicial + 3);
  //Agrega el signo
  if (signo == 1) {
    dato = -1 * constrVar(varr1, varr2, varrDec);
    return dato;
  }
  else {
    dato = constrVar(varr1, varr2, varrDec);
    return dato;
  }
}

void guardarDatoFloat(float dato, int dirPagInicial) {
  //Funcion que guarda datos flotantes de la memoria eeprom,
  byte varr1; //primeros 8 bits para guardar en la EEPROM (LSB)
  byte varr2; //ultimo 8 bits para guardar en la EEPROM (MSB)
  byte varrDec; //parte decimal para guardar en la EEPROM
  byte signo; // signo del numero
  float dato2;
  if (dato < 0.0) {
    signo = 1;
    dato2 = (-1 * dato);
  }
  else {
    signo = 0;
    dato2 = dato;
  }
  transfVar(dato2, varr1, varr2, varrDec);
  eepromEscribe(dirEEPROM, dirPagInicial, varr1);
  delay(100);
  eepromEscribe(dirEEPROM, dirPagInicial + 1, varr2);
  delay(100);
  eepromEscribe(dirEEPROM, dirPagInicial + 2, varrDec);
  delay(100);
  eepromEscribe(dirEEPROM, dirPagInicial + 3, signo);
  delay(100);
}

/// @brief Reinicia variables del avance para evitar problemas
void resetAvance(){
  distLinealRuedaDerecha = 0;
  distLinealRuedaDerechaAnterior = 0;

  distLinealRuedaIzquierda = 0;
  distLinealRuedaIzquierdaAnterior = 0;
}
