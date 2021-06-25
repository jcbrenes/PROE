//Código de locomoción y comunicacion para el proyecto PROE
//Usado en la placa Feather M0 RFM95
//https://github.com/jcbrenes/PROE

#include<Wire.h> //Bilbioteca para la comunicacion I2C
#include "wiring_private.h" // Necesario para el I2C secundario y usar pinPeripheral() function
#include <SPI.h> //Biblioteca para la comunicacion por radio frecuencia
#include <RH_RF69.h> //Biblioteca para la comunicacion por radio frecuencia

//Variables del enjambre para la comunicación a la base
uint8_t cantidadRobots = 3; //Cantidad de robots en enjambre. No cuenta la base, solo los que hablan.
unsigned long idRobot = 1; //ID del robot, este se usa para ubicar al robot dentro de todo el ciclo de TDMA.

//constantes del robot empleado
const int tiempoMuestreo=10000; //unidades: micro segundos
const float pulsosPorRev=206.0; //cantidad de pulsos de una única salida
const int factorEncoder=2; //cantidad de salidas de encoders que se están detectando (pueden ser 2 o 4)
const float circunferenciaRueda=139.5;//Circunferencia de la rueda = 139.5mm 
const float pulsosPorMilimetro=((float)factorEncoder*pulsosPorRev)/circunferenciaRueda; 
const float distanciaCentroARueda=63.7;// Radio de giro del carro, es la distancia en mm entre el centro y una rueda. 
const float conversionMicroSaMin=1/(60 * 1000000);// factor de conversion microsegundo (unidades del tiempo muestreo) a minuto
const float conversionMicroSaSDiv=1000000;// factor de conversion microsegundo (unidades del tiempo muestreo) a segundo
const float tiempoMuestreoS= (float)tiempoMuestreo/conversionMicroSaSDiv;

//constantes para control PID de velocidad (están unidas con la constante de tiempo por simplificación de la ecuación)
const float velRequerida=170.0; //unidades mm/s
const float KpVel=0.90; //constante control proporcional
const float KiVel=1.0 * tiempoMuestreoS; //constante control integral
const float KdVel=0.01 / tiempoMuestreoS ; //constante control derivativo

//constantes para control PID de giro 
const float KpGiro=5.0; //constante control proporcional
const float KiGiro=20.0 * tiempoMuestreoS;//constante control integral
const float KdGiro=0.08 / tiempoMuestreoS; //constante control derivativo

//Constantes para la implementación del control PID real
const int errorMinIntegral=-250;
const int errorMaxIntegral=250;
const int limiteSuperiorCicloTrabajoVelocidad=200;
const int limiteInferiorCicloTrabajoVelocidad=-200;
const int limiteSuperiorCicloTrabajoGiro=110;
const int limiteInferiorCicloTrabajoGiro=-110;
const int cicloTrabajoMinimo= 20;
const int minCiclosEstacionario= 20;

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
const int longitudArregloObstaculos = 1000;

//Constantes algoritmo exploración
const int unidadAvance= 400; //medida en mm que avanza cada robot por movimiento
const int unidadRetroceso= -100; //medida en mm que retrocede el robot al encontrar un obstáculo
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
enum PosiblesEstados {AVANCE=0, RETROCEDA, GIRE_DERECHA, GIRE_IZQUIERDA, ESCOGER_DIRECCION, GIRO, NADA};
char *PosEstados[] = {"AVANCE", "RETROCEDA", "GIRE_DERECHA", "GIRE_IZQUIERDA","ESCOGER_DIRECCION","GIRO", "NADA"};
PosiblesEstados estado = AVANCE;

//variable que almacena el tiempo del último ciclo de muestreo
unsigned long tiempoActual = 0;
unsigned long tiempoMaquinaEstados = 0;
unsigned long tiempoPollingPeri = 0;

//contadores de pulsos del encoder
int contPulsosDerecha = 0;
int contPulsosIzquierda = 0;
int contPulsosDerPasado = 0;
int contPulsosIzqPasado = 0;

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

//Variables para las interrupciones de los encoders
byte estadoEncoderDer = 1;
byte estadoEncoderIzq = 1;

//Variables para el almacenar datos de obstáculos
int datosSensores[longitudArregloObstaculos][6]; //Arreglo que almacena la información de obstáculos de los sensores (tipo sensor, distancia, ángulo)
int ultimoObstaculo = -1; //Apuntador al último obstáculo en el arreglo. Se inicializa en -1 porque la función de guardar aumenta en 1 el índice
int minEnviado = 0; //Almacena el recorrido de la lista de obstaculos
bool nuevoObstaculo = false; //Bandera verdadera cuando se detecta un nuevo obstaculo
unsigned long tiempoSTM = 0; //Almacena el tiempo de muestreo del STM


//Variables para el algoritmo de exploración
int distanciaAvanzada = 0;
int poseActual[3] = {0, 0, 0}; //Almacena la pose actual: ubicación en x, ubicación en y, orientación.
bool giroTerminado = 1; //Se hace esta variable global para saber cuando se está en un giro y cuando no
bool obstaculoAdelante = false;
bool obstaculoDerecha = false;
bool obstaculoIzquierda = false;
enum orientacionesRobot {ADELANTE = 0, DERECHA = 90, IZQUIERDA = -90, ATRAS = 180}; //Se definen las orientaciones de los robots, el número indica la orientación de la pose
orientacionesRobot direccionGlobal = ADELANTE; //Se inicializa Adelante, pero en Setup se asignará un valor random
int anguloGiro = 0;

//Variables para el magnetómetro y su calibración
const float declinacionMag = 0.0; //correccion del campo magnetico respecto al norte geográfico en Costa Rica
const float alfa = 1; //constante para filtro de datos
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
float ayft, gzft;

//Variables para la comunicación por radio frecuencia
#define RF69_FREQ      915.0  //La frecuencia debe ser la misma que la de los demas nodos.
#define DEST_ADDRESS   10     //No sé si esto es totalmente necesario, creo que no porque nunca usé direcciones.
#define MY_ADDRESS     4      //Dirección de este nodo. La base la usa para enviar el reloj al inicio
//Definición de pines. Creo que no todos se están usando, me parece que el LED no.
#define RFM69_CS       8
#define RFM69_INT      3
#define RFM69_RST      4
RH_RF69 rf69(RFM69_CS, RFM69_INT); // Inicialización del driver de la biblioteca Radio Head.

//Variables para el mensaje que se va a transmitir a la base
uint8_t buf[RH_RF69_MAX_MESSAGE_LEN]; //Buffer para recibir mensajes.
bool timeReceived = false; //Bandera para saber si ya recibí el clock del máster.
unsigned long tiempoRobotTDMA = 50; //Slot de tiempo que tiene cada robot para hablar. Unidades ms.
unsigned long tiempoCicloTDMA = cantidadRobots * tiempoRobotTDMA; //Duración de todo un ciclo de comunicación TDMA. Unidades ms.
unsigned long timeStamp1; //Estampa de tiempo para eliminar el desfase por procesamiento del reloj al recibirse. Se obtiene apenas se recibe el reloj.
unsigned long timeStamp2; //Estampa de tiempo para eliminar el desfase por procesamiento del reloj al recibirse. Se obtiene al guardar en memoria el reloj.
unsigned long data; //Variable donde se almacenará el valor del reloj del máster.
volatile bool mensajeCreado = false; //Bandera booleana para saber si ya debo crear otro mensaje cuando esté en modo transmisor.
uint8_t mensaje[7 * sizeof(float)]; //El mensaje a enviar. Llevará 6 datos tipo float: (robotID, xP, yP, phi, tipoObs, rObs, alphaObst)
uint32_t* ptrMensaje; //Puntero para descomponer el float en bytes.
const uint8_t timeOffset = 2; //Offset que existe entre el máster enviando y el nodo recibiendo.

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
  pinMode(INT_OBSTACULO,OUTPUT);
  pinMode(ENC_DER_C1, INPUT); 
  pinMode(ENC_DER_C2, INPUT); 
  pinMode(ENC_IZQ_C1, INPUT);
  pinMode(ENC_IZQ_C2, INPUT);

   //asignación de interrupciones
  delay(3000); //delay para evitar interrupciones al arrancar
  //attachInterrupt(ENC_DER_C1, PulsosRuedaDerechaC1,CHANGE);  //conectado el contador C1 rueda derecha
  attachInterrupt(ENC_DER_C2, PulsosRuedaDerechaC2,CHANGE);
  //attachInterrupt(ENC_IZQ_C1, PulsosRuedaIzquierdaC1,CHANGE);  //conectado el contador C1 rueda izquierda
  attachInterrupt(ENC_IZQ_C2, PulsosRuedaIzquierdaC2,CHANGE);
  attachInterrupt(digitalPinToInterrupt(INT_OBSTACULO), DeteccionObstaculo,FALLING);
  
  //temporización y varibales aleatorias
  tiempoActual=millis(); //para temporización de los ciclos
  tiempoMaquinaEstados= micros();
  tiempoPollingPeri= micros();
  randomSeed(analogRead(A5)); //Para el algoritmo de exploración, el pinA5 está al aire
  direccionGlobal = (orientacionesRobot)(random(-1, 2) * 90); //Se asigna aleatoriamente una dirección global a seguir por el algoritmo RWD
  
  //***Inicialización de puertos seriales***
  Serial.begin(115200); //Puerto serie para comunicación con la PC
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

  if (!rf69.init()) {
    Serial.println("RFM69 inicialización fallida");
    while (1);
  }
  // Setear frecuencia
  if (!rf69.setFrequency(RF69_FREQ)) {
    Serial.println("setFrequency failed");
  }

  rf69.setTxPower(20, true); // Configura la potencia
  uint8_t key[] = { 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
                    0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08
                  };

  rf69.setEncryptionKey(key);
  rf69.setPromiscuous(true);
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

  Serial.println("¡RFM69 radio en funcionamiento!");
  delay(20);
}

void loop(){

  //******En caso de usar el robot solo (no como enjambre), comentar la siguiente linea
  //sincronizacion(); //Esperar mensaje de sincronizacion de la base antes de moverse

  //***POLLING*** Acciones que se ejecutan periodicamente. Más frecuentemente que la máquina de estados
  //Revisión de los encoders de los motores (tipo polling para no afectar la comunicación con Ints)
  //revisaEncoders(); 
  RecorrerObstaculos();



  //***MAQUINA DE ESTADOS*** Donde se manejan los comportamientos del robot y el control PID
  //Las acciones de la máquina de estados y los controles se efectuarán en tiempos fijos de muestreo
  if((micros()-tiempoMaquinaEstados)>=tiempoMuestreo){
     tiempoMaquinaEstados=micros();
     tiempoActual=millis();//tomo el tiempo actual (para reducir las llamadas a millis)

     //Polling de perifericos conectados a los buses I2C
     leerMsgSTM(); //lectura de obstáculo en el STM
     ultimoAngMagnet= medirMagnet(); //medición de la orientación con el magnetómetro
     detectaCambio(ultimoAngMagnet); //detectar cambio de orientación para corrección en valores
     leeMPU(anguloMPU,velMPU); //medición del MPU
     
     //Máquina de estados que cambia el modo de operación
     switch (estado) {
  
        case AVANCE:  { 
          bool avanceTerminado= AvanzarDistancia(unidadAvance); 
          //bool avanceTerminado= true; 
          if (avanceTerminado){
            ActualizarUbicacionReal(); //Actualiza la ubicación actual en base al avance anterior y la orientación actual
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
            ActualizarUbicacionReal(); //Actualiza la ubicación actual en base al avance anterior y la orientación actual
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
          RevisaObstaculoPeriferia(); //Revisa los osbtáculos presentes en la pose actual
          AsignarDireccionRWD(); //Asigna un ángulo de giro en base al algoritmo Random Walk con Dirección
          resetVarGiro(); //resetea las variables globales utilizadas para medir el giro real
          
          //Para evitar estado de giro permanente
          if(estado!=GIRO){ //Si no se encontraba en giro almacenar el tiempoGiro
            tiempoGiro=millis(); //Almacena el tiempo cuando se cambio a giro para comparar con el limite
          }
          
          estado= GIRO;
        }
  
        case GIRO: {
          giroTerminado=Giro((float)anguloGiro);


          if ((millis() - tiempoGiro) > limiteGiro){ //Si pasa mas del limite de tiempo tratando de girar se detiene y lo trata como si hubiera completado el giro, evita que se quede intentando girar si está bloqueado
            giroTerminado = true;
          }
          
          if(giroTerminado){
            ConfiguracionParar();
            //Serial.print("Giro real: "); Serial.println(angActualRobot);
            estado=AVANCE;
          }
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
  
  if (tipoSensor!=0){ //Si el sensor sigue en 0 significa que el STM no envió nada
    CrearObstaculo(tipoSensor, distancia, angulo);
  }

}



void CrearObstaculo(int tipoSensor, int distancia, int angulo){ 
//Agrega un nuevo onstaculo a la lista, usa la poseActual global cuando se llama
  if (ultimoObstaculo == longitudArregloObstaculos - 1) { //si llega al final del arreglo regresa al inicio, sino suma 1
    ultimoObstaculo = 0;
  } else {
    ultimoObstaculo++;
  }
  datosSensores[ultimoObstaculo][0] = poseActual[0]; //Guarda la pose actual donde se detectó el obstáculo
  datosSensores[ultimoObstaculo][1] = poseActual[1];
  datosSensores[ultimoObstaculo][2] = poseActual[2];
  datosSensores[ultimoObstaculo][3] = tipoSensor;
  datosSensores[ultimoObstaculo][4] = distancia;
  datosSensores[ultimoObstaculo][5] = angulo;

  nuevoObstaculo = true;
}



void CrearMensaje(int poseX, int poseY, int rotacion, int tipoSensorX, int distanciaX, int anguloX){ 
//Crea el mensaje a ser enviado por comunicación RF. Es usado en la interrupcion del RTC
  
  if(timeReceived && !mensajeCreado){       //Aquí solo debe enviar mensajes, pero eso lo hace la interrupción, así que aquí se construyen mensajes y se espera a que la interrupción los envíe.

    float frac = 0.867;                     //Fracción que se usa para insertarle a los random decimales. Se escogió al azar, el número no significa nada.

    //Genero números al azar acorde a lo que el robot eventualmente podría enviar
    float robotID = (float)idRobot;         //Esta variable se debe volver a definir, pues idRobot al ser global presenta problema al crear el mensaje.
    float xP = (float)poseX;                //Posición X en que se detecto el obstaculo
    float yP = (float)poseY;                //Posición Y en que se detecto el obstaculo
    float phi = (float)rotacion;            //"Orientación" del robot
    float tipo = (float)tipoSensorX;        //Tipo de sensor que se detecto (Sharp, IR Fron, IR Der, IR Izq, Temp, Bat)
    float rObs = (float)distanciaX;         //Distancia medida por el sharp si aplica
    float alphaObs = (float)(anguloX);      //Angulo respecto al "norte" (orientación inicial del robot medida por el magnetometro)

    //Construyo mensaje (es una construcción bastante manual que podría mejorar)
    ptrMensaje = (uint32_t*)&robotID;       //Utilizo el puntero para extraer la información del dato flotante.
    for (uint8_t i = 0; i < 4; i++) {
      mensaje[i] = (*ptrMensaje & (255UL << i * 8)) >> i * 8; //La parte de "(255UL << i*8)) >> i*8" es solo para ir acomodando los bytes en el array de envío mensaje[].
    }

    ptrMensaje = (uint32_t*)&xP;
    for (uint8_t i = 4; i < 8; i++) {
      mensaje[i] = (*ptrMensaje & (255UL << (i - 4) * 8)) >> (i - 4) * 8;
    }

    ptrMensaje = (uint32_t*)&yP;
    for (uint8_t i = 8; i < 12; i++) {
      mensaje[i] = (*ptrMensaje & (255UL << (i - 8) * 8)) >> (i - 8) * 8;
    }

    ptrMensaje = (uint32_t*)&phi;
    for (uint8_t i = 12; i < 16; i++) {
      mensaje[i] = (*ptrMensaje & (255UL << (i - 12) * 8)) >> (i - 12) * 8;
    }

    ptrMensaje = (uint32_t*)&tipo;
    for (uint8_t i = 16; i < 20; i++) {
      mensaje[i] = (*ptrMensaje & (255UL << (i - 16) * 8)) >> (i - 16) * 8;
    }

    ptrMensaje = (uint32_t*)&rObs;
    for (uint8_t i = 20; i < 24; i++) {
      mensaje[i] = (*ptrMensaje & (255UL << (i - 20) * 8)) >> (i - 20) * 8;
    }

    ptrMensaje = (uint32_t*)&alphaObs;
    for (uint8_t i = 24; i < 28; i++) {
      mensaje[i] = (*ptrMensaje & (255UL << (i - 24) * 8)) >> (i - 24) * 8;
    }

    //Una vez creado el mensaje, no vuelvo a crear otro hasta que la interrupción baje la bandera.
    mensajeCreado = true;
  }
}

void RecorrerObstaculos(){ 
//Recorre la lista de obstaculos empezando desde el ultimo recibido, permite enviar mensajes anteriores cuando el robot no está detectando nada
  if(!mensajeCreado){ //Verifica que se envio el ultimo mensaje creado para no recorrer la lista sin necesidad
    if(nuevoObstaculo){ //Si se recibio un nuevo obstaculo reiniciar el recorrido de la lista desde ultimoObstaculo
      nuevoObstaculo = false;
      minEnviado = ultimoObstaculo;
    }
    
    else{
      if(minEnviado>0){
        minEnviado = minEnviado -1;
      }
    }
      
    CrearMensaje(datosSensores[minEnviado][0], datosSensores[minEnviado][1], datosSensores[minEnviado][2], datosSensores[minEnviado][3], datosSensores[minEnviado][4], datosSensores[minEnviado][5]);
   }
}


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

  //Calcular nueva posición basado en la distancia y el angulo en que se movio (convertir coordenadas polares a rectangulares)
  poseActual[0] = poseActual[0] + (distanciaAvanzada * cos(orientacion)); //coordenada X
  poseActual[1] = poseActual[1] + (distanciaAvanzada * sin(orientacion)); //coordenada Y

}

void DeteccionObstaculo(){
//Función tipo interrupción llamada cuando se activa el pin de detección de obstáculo del STM32
//Son obstáculos que requieren que el robot retroceda y cambie de dirección inmediatamente

  if(estado!=RETROCEDA && giroTerminado==1 && tiempoActual>2000){
    //Solo si el estado ya no es retroceder.  También que no está haciendo un giro 
    //y para que ignore las interrupciones los primeros segundos al encenderlo
    estado=RETROCEDA;
    ActualizarUbicacionReal(); //Como se interrumpió un movimiento, actualiza la ubicación actual
    ConfiguracionParar(); //Se detiene un momento y reset de encoders 
    tiempoRetroceso= tiempoActual; //Almacena el ultimo tiempo para usarlo en el temporizador
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
  } else {
    //La ecuación del ángulo de giro toma en cuenta todas las restricciones anteriores y lo pasa a escala de grados (nomenclatura de orientación)
    anguloGiro = random(minRandom, maxRandom) * incrementoPosible - 90;
  }

  //Reset de la simplificación sobre obstáculo en la pose actual
  obstaculoAdelante = false;
  obstaculoDerecha = false;
  obstaculoIzquierda = false;
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
  sumErrorGiro += errorGiro;
  sumErrorGiro = constrain(sumErrorGiro, errorMinIntegral, errorMaxIntegral);

  //error derivativo (diferencial)
  float difErrorGiro = (errorGiro - errorAnteriorGiro);

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

  float factorFus=0.7;
  angActualRobot=GiroReal(anguloMPU,anguloInicial,ultimoAngMagnet); //calcula el giro real
  
  posActualRuedaDerecha = ConvDistAngular(contPulsosDerecha)*(1-factorFus)+factorFus*angActualRobot;

  int cicloTrabajoRuedaDerecha = ControlPosGiroRueda( grados, posActualRuedaDerecha, sumErrorGiroDer, errorAnteriorGiroDer );

  posActualRuedaIzquierda = ConvDistAngular(contPulsosIzquierda)*(1-factorFus)+factorFus*angActualRobot;
  int cicloTrabajoRuedaIzquierda = ControlPosGiroRueda( -grados, -posActualRuedaIzquierda, sumErrorGiroIzq, errorAnteriorGiroIzq);

  ConfiguraEscribePuenteH (cicloTrabajoRuedaDerecha, cicloTrabajoRuedaIzquierda);

  bool giroListo = EstadoEstacionario (cicloTrabajoRuedaDerecha, cicloTrabajoRuedaIzquierda, contCiclosEstacionarioGiro);

  return giroListo;

}


//***Funciones para configuración del Puente H y por ende la dirección del robot

void ConfiguracionParar() {
  //Deshabilita las entradas del puente H, por lo que el carro se detiene
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, LOW);
  ResetContadoresEncoders();
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

void revisaEncoders(){
//Función que lee los estados de las entradas de los encoders y en caso de cambios actualiza el contador de pulsos
//Se llama cuando se revisan los encoders en polling y no en interrupciones 
  LecturaEncoder(ENC_DER_C1, ENC_DER_C2, estadoEncoderDer, contPulsosDerecha); //Revisa los encoders rueda izquierda
  LecturaEncoder(ENC_IZQ_C1, ENC_IZQ_C2, estadoEncoderIzq, contPulsosIzquierda); //Revisa los encoders rueda derecha
}

void PulsosRuedaDerechaC1(){
//Manejo de interrupción del canal C1 del encoder de la rueda derecha
  //LecturaEncoder(ENC_DER_C1, ENC_DER_C2, estadoEncoderDer, contPulsosDerecha);
  LecturaEncoder2(ENC_DER_C1, ENC_DER_C2, estadoEncoderDer, contPulsosDerecha);

}

void PulsosRuedaDerechaC2() {
  //Manejo de interrupción del canal C2 del encoder de la rueda derecha
  //LecturaEncoder(ENC_DER_C1, ENC_DER_C2, estadoEncoderDer, contPulsosDerecha);
  LecturaEncoder2(ENC_DER_C1, ENC_DER_C2, estadoEncoderDer, contPulsosDerecha);
}

void PulsosRuedaIzquierdaC1() {
  //Manejo de interrupción del canal C1 del encoder de la rueda izquierda
  //LecturaEncoder(ENC_IZQ_C1, ENC_IZQ_C2, estadoEncoderIzq, contPulsosIzquierda);
  LecturaEncoder2(ENC_IZQ_C1, ENC_IZQ_C2, estadoEncoderIzq, contPulsosIzquierda);
}

void PulsosRuedaIzquierdaC2() {
  //Manejo de interrupción del canal C2 del encoder de la rueda izquierda
  //LecturaEncoder(ENC_IZQ_C1, ENC_IZQ_C2, estadoEncoderIzq, contPulsosIzquierda);
  LecturaEncoder2(ENC_IZQ_C1, ENC_IZQ_C2, estadoEncoderIzq, contPulsosIzquierda);
}

void LecturaEncoder(int entradaA, int entradaB, byte& state, int& contGiro) {
  //Función que determina si el motor está avanzando o retrocediendo en función del estado de las salidas del encoder y el estado anterior
  //Modifica la variable contadora de pulsos de ese motor
  //Funciona si se están revisando las 2 salidas de cada encoder (factor 4)

  //se almacena el valor actual del estado
  byte statePrev = state;

  //lectura de los encoders
  int A = digitalRead(entradaA);
  int B = digitalRead(entradaB);

  //se define el nuevo estado
  if ((A == HIGH) && (B == HIGH)) state = 1;
  if ((A == HIGH) && (B == LOW)) state = 2;
  if ((A == LOW) && (B == LOW)) state = 3;
  if ((A == LOW) && (B == HIGH)) state = 4;

  //Se aumenta o decrementa el contador de giro en base al estado actual y el anterior
  switch (state)
  {
    case 1:
      {
        if (statePrev == 2) contGiro--;
        if (statePrev == 4) contGiro++;
        break;
      }
    case 2:
      {
        if (statePrev == 1) contGiro++;
        if (statePrev == 3) contGiro--;
        break;
      }
    case 3:
      {
        if (statePrev == 2) contGiro++;
        if (statePrev == 4) contGiro--;
        break;
      }
    default:
      {
        if (statePrev == 1) contGiro--;
        if (statePrev == 3) contGiro++;
      }
  }
}

void LecturaEncoder2(int entradaA, int entradaB, byte& state, int& contGiro) { //Funcion de lectura de encoders con un solo interrupt en C2
  //Función que determina si el motor está avanzando o retrocediendo en función del estado de las salidas del encoder y el estado anterior
  //Modifica la variable contadora de pulsos de ese motor
  //Funciona si se están revisando solo 1 salida de cada encoder (factor 2)

  //se almacena el valor actual del estado
  byte statePrev = state;

  //lectura de los encoders
  int A = digitalRead(entradaA); //C1
  int B = digitalRead(entradaB); //C2

  //se define el nuevo estado
  if ((A == HIGH) && (B == HIGH)) state = 1;
  if ((A == HIGH) && (B == LOW)) state = 2;
  if ((A == LOW) && (B == LOW)) state = 3;
  if ((A == LOW) && (B == HIGH)) state = 4;

  //Se aumenta o decrementa el contador de giro en base al estado actual y el anterior
  switch (state)
  {
    case 1:
      {
        if (statePrev == 3) contGiro--;
        if (statePrev == 2) contGiro++;
        break;
      }
    case 2:
      {
        if (statePrev == 4) contGiro++;
        if (statePrev == 1) contGiro--;
        break;
      }
    case 3:
      {
        if (statePrev == 4) contGiro++;
        if (statePrev == 1) contGiro--;
        break;
      }
    default:
      {
        if (statePrev == 3) contGiro--;
        if (statePrev == 2) contGiro++;
      }
  }
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

  float distLinealRuedaDerecha = (float)contPulsosDerecha / pulsosPorMilimetro;
  float distLinealRuedaIzquierda = - (float)contPulsosIzquierda / pulsosPorMilimetro;
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
  sumErrorVel += errorVel;
  sumErrorVel = constrain(sumErrorVel, errorMinIntegral, errorMaxIntegral);

  //error derivativo (diferencial)
  float difErrorVel = (errorVel - errorAnteriorVel);

  //ecuación de control PID
  float pidTermVel = (KpVel * errorVel) + (KiVel * sumErrorVel) + (KdVel * difErrorVel)/5;  //Agregada división entre 5 porque PID saturaba, buscar donde está el error

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
  //Avanza hacia adelante una distancia definida en mm a velocidad constante
  //Devuelve true cuando alcanzó la distancia deseada

  float velSetPoint= velRequerida;
  if (distanciaDeseada<0){  //Si la distancia deseada es negativa, significa retroceder y por ende velocidad negativa (y un poco más despacio)
    velSetPoint= -1 * velRequerida * 0.75;
  }

  velActualDerecha= calculaVelocidadRueda(contPulsosDerecha, contPulsosDerPasado);
  int cicloTrabajoRuedaDerecha = ControlVelocidadRueda(velSetPoint, velActualDerecha, sumErrorVelDer, errorAnteriorVelDer);

  velActualIzquierda= -1.0 * calculaVelocidadRueda(contPulsosIzquierda, contPulsosIzqPasado); //como las ruedas están en espejo, la vel es negativa cuando avanza, por eso se invierte
  int cicloTrabajoRuedaIzquierda = ControlVelocidadRueda(velSetPoint, velActualIzquierda, sumErrorVelIzq, errorAnteriorVelIzq);
   
  ConfiguraEscribePuenteH (cicloTrabajoRuedaDerecha, cicloTrabajoRuedaIzquierda);

  float distanciaAvanzada= calculaDistanciaLinealRecorrida();

  bool avanceListo = false; 
  if (abs(distanciaAvanzada) >= abs(distanciaDeseada)) {
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




//*****Funciones para manejar la orientación*********

void resetVarGiro() {
  //Funcion que resetea las variables globales de la función GiroReal
  //cuadranteAnterior = NULL;
  cuadranteActual = NULL;
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

      int cuadranteActual=buscaCuadrante(angulo); 

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
     cuadranteAnterior=cuadranteActual; 
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
    float facFus= 0.5; //factor de cuanto pesa el valor del MPU (valor de 0 a 1)
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

//*****Funciones del magnetómetro*******

void origenMagnet(){ //Mide la orientación y guarda un promedio
  float x=0;
  for(int i=0; i<25; i++){ //Medir orientacion con el magnetometro 25 veces y sacar un promedio
    x = x + medirMagnet();
  }
  magInicioOrigen = x/25;
}

void inicializaMagnet() {
  //Funcion que establece la comunicacion con el magnetometro, lo setea para una frecuencia de muestreo de 200 Hz
  //y en modo de medición continua
  segundoI2C.beginTransmission(dirMag); //
  segundoI2C.write(0x0B); // Inicialización
  segundoI2C.write(0x01); // 
  segundoI2C.endTransmission();
  segundoI2C.beginTransmission(dirMag); //
  segundoI2C.write(0x09); // Modo: Continuously Measure, 200Hz, 8 Gauss, OSR 512
  segundoI2C.write(0x1D); // Set the Register
  segundoI2C.endTransmission();

  //Carga los valores de calibración del magnetometro
  xoff = leerDatoFloat(0);
  yoff = leerDatoFloat(4);
  angElips = leerDatoFloat(8);
  factorEsc = leerDatoFloat(12);

  //Obtiene el valor inicial de ángulo
  origenMagnet();
}

float medirMagnet() {
  //Funcion que extrae los datos crudos del magnetometro, carga los valores de calibracion
  //adicionalmente usa un filtro para la toma de datos (media movil)
  //retorna el angulo en un rango de [0,+-180]
  short x, y, z;
  float xof, yof, xrot, yrot, xf, yf;
  //establece comunicación con el magnetómetro
    
    segundoI2C.beginTransmission(dirMag);
    segundoI2C.write(0x00); //start with register 3.
    segundoI2C.endTransmission();
    //Pide 6 bytes del registro del magnetometro
    segundoI2C.requestFrom(dirMag, 6);
    if (6 <= segundoI2C.available()) {
      x = segundoI2C.read(); //LSB  x
      x |= segundoI2C.read() << 8; //MSB  x
      y = segundoI2C.read(); //LSB  y
      y |= segundoI2C.read() << 8; //MSB y
      z = segundoI2C.read(); //LSB z
      z |= segundoI2C.read() << 8; //MSB z
    }
    //Se puede hacer un filtro de media entre los datos
    xft = x * alfa + (1 - alfa) * xft;
    yft = y * alfa + (1 - alfa) * yft;

  //***Corrección con los datos de calibración  
  //Sustrae los offset
  xof = xft - xoff;
  yof = yft - yoff;
  //Rotación y escalamiento de los datos
  xrot = xof * cos(-angElips) - yof * sin(-angElips);
  yrot = xof * sin(-angElips) + yof * cos(-angElips);
  if (factorEsc > 0) {
    xf = cos(angElips) * xrot - yrot * sin(angElips) * factorEsc;
    yf = sin(angElips) * xrot + cos(angElips) * yrot * factorEsc;
  }
  else {
    xf = -1 * cos(angElips) * xrot * factorEsc - yrot * sin(angElips);
    yf = -1 * sin(angElips) * xrot * factorEsc + cos(angElips) * yrot;
  }

  //Obtiene el alguno con respecto al norte
  float angulo = atan2(yf, xf);
  angulo = angulo * (180 / PI); //convertimos de Radianes a grados
  //Mostramos el angulo entre el eje X y el Norte
  if (angulo < 0.0) {
    angulo = angulo + 360;
  }
  
  return angulo;
}





/**** RTC FUNCIONES****/

inline bool RTCisSyncing() {
  //Función que lee el bit de sincronización de los registros
  return (RTC->MODE0.STATUS.bit.SYNCBUSY);
}

void sincronizacion() {
  while (!timeReceived) { //Espera mensaje de sincronización de la base
    uint8_t len = sizeof(buf);                                                  //Obtengo la longitud máxima del mensaje a recibir
    if (rf69.recv(buf, &len, timeStamp1)) {                                     //Llamo a recv() si recibí algo. El timeStamp1 guarda el valor del RTC del nodo en el momento en que comienza a procesar el mensaje.
      Serial.println("¡Reloj del máster clock recibido!");
      buf[len] = 0;                                                             //Limpio el resto del buffer.
      data = buf[3] << 24 | buf[2] << 16 | buf[1] << 8 | buf[0];                //Construyo el dato con base en el buffer y haciendo corrimiento de bits.
      timeStamp2 = RTC->MODE0.COUNT.reg;                                        //Una vez procesado el mensaje del reloj del máster, guardo otra estampa de tiempo para eliminar este tiempo de procesamiento.
      unsigned long masterClock = data + (timeStamp2 - timeStamp1) + timeOffset; //Calculo reloj del master como: lo enviado por el máster (data) + lo que dura el mensaje en llegarme (timeOffset) + lo que duré procesando el mensaje (tS2-tS1).
      RTC->MODE0.COUNT.reg = masterClock;                                       //Seteo el RTC del nodo al tiempo del master.
      RTC->MODE0.COMP[0].reg = masterClock + idRobot * tiempoRobotTDMA + 50;    //Partiendo del clock del master, calculo la próxima vez que tengo que hacer la interrupción según el ID. Sumo 50 ms para dejar un colchón que permita que todos los robots oigan el mensaje antes de empezar a hablar.
      while (RTCisSyncing());                                                   //Espero la sincronización.

      timeReceived = true;                                                      //Levanto la bandera que indica que recibí el reloj del máster y ya puedo pasar a transmitir.
      //Wire.onReceive(RecibirI2C);
    }
  }
}

void config32kOSC() {
  //Función que configura el reloj de cristal, en caso de querer usar este reloj. Se debe estudiar más a detalle los comandos.
#ifndef CRYSTALLESS
  SYSCTRL->XOSC32K.reg = SYSCTRL_XOSC32K_ONDEMAND |
                         SYSCTRL_XOSC32K_RUNSTDBY |
                         SYSCTRL_XOSC32K_EN32K |
                         SYSCTRL_XOSC32K_XTALEN |
                         SYSCTRL_XOSC32K_STARTUP(6) |
                         SYSCTRL_XOSC32K_ENABLE;
#endif
}

void configureClock() {
  //Función que configura el clock. Se debe estudiar más a detalle los comandos.
  GCLK->GENDIV.reg = GCLK_GENDIV_ID(2) | GCLK_GENDIV_DIV(4);
  while (GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY);

#ifdef CRYSTALLESS
  GCLK->GENCTRL.reg = (GCLK_GENCTRL_GENEN | GCLK_GENCTRL_SRC_OSCULP32K | GCLK_GENCTRL_ID(2) | GCLK_GENCTRL_DIVSEL );
#else
  GCLK->GENCTRL.reg = (GCLK_GENCTRL_GENEN | GCLK_GENCTRL_SRC_XOSC32K | GCLK_GENCTRL_ID(2) | GCLK_GENCTRL_DIVSEL );
#endif

  while (GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY);
  GCLK->CLKCTRL.reg = (uint32_t)((GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK2 | (RTC_GCLK_ID << GCLK_CLKCTRL_ID_Pos)));
  while (GCLK->STATUS.bit.SYNCBUSY);
}

void RTCdisable() {
  //Función que deshabilita el RTC.
  RTC->MODE0.CTRL.reg &= ~RTC_MODE0_CTRL_ENABLE; // disable RTC
  while (RTCisSyncing());
}

void RTCenable() {
  //Función que habilita el RTC.
  RTC->MODE0.CTRL.reg |= RTC_MODE0_CTRL_ENABLE; // enable RTC
  while (RTCisSyncing());
}

void RTCreset() {
  //Función que resetea el RTC.
  RTC->MODE0.CTRL.reg |= RTC_MODE0_CTRL_SWRST; // software reset
  while (RTCisSyncing());
}

void RTCresetRemove() {
  //Función que quita el reset del RTC.
  RTC->MODE0.CTRL.reg &= ~RTC_MODE0_CTRL_SWRST; // software reset remove
  while (RTCisSyncing());
}

void RTC_Handler(void) {
  //Vector de interrupción.
  if (RTC->MODE0.COUNT.reg > 0x00000064) {      //Si el valor del contador es mayor a 0x64 (100 en decimal) entonces haga la interrupción. Esto para evitar el problema que la interrupción se llame al puro inicio. No sé por qué pasaba esto, investigar más el tema.
    RTC->MODE0.COMP[0].reg += tiempoCicloTDMA;  //Quiero que haga una interrupción en el próximos ciclo del TDMA, actualizo el nuevo valor a comparar.  **¿Por qué debo agregar el [0]?**
    while (RTCisSyncing());                     //Llamo a función de escritura.

    rf69.send(mensaje, sizeof(mensaje));        //Llamo a la función de enviar de la biblioteca Radio Head para enviar el mensaje del nodo.
    mensajeCreado = false;                      //Bajo la bandera para indicar que ya se envió el mensaje y se puede crear uno nuevo.

    RTC->MODE0.INTFLAG.bit.CMP0 = true;         //Limpiar la bandera de la interrupción.
  }
}


void inicializarMPU() {
//Inicializa la comunicación con el MPU
  segundoI2C.beginTransmission(0x68); //empezar comunicacion con el mpu6050
  segundoI2C.write(0x6B);   //escribir en la direccion 0x6B
  segundoI2C.write(0x00);   //escribe 0 en la direccion 0x6B (arranca el sensor)
  segundoI2C.endTransmission();   //termina escritura

  //Carga los valores de calibración del MPU6050
  gx_off = leerDatoFloat(16);
  gy_off = leerDatoFloat(20);
  gz_off = leerDatoFloat(24);
  acx_off = leerDatoFloat(28);
  acy_off = leerDatoFloat(32);
  acz_off = leerDatoFloat(36);
}

void resetVarMPU(){
//Reset de variables del MPU
  tiempoPrev=0;
  gir_ang_zPrev=0;
  }

void leeMPU(float &gir_ang_z, float &vely) {
//Funcion que extrae los datos crudos del MPU, los calibra y calcula desplazamiento angulares
  int16_t gyro_x, gyro_y, gyro_z, tmp, ac_x, ac_y, ac_z; //guardan los datos crudos
  float gz, ay; //guardan los valores reales de aceleración y velocidad angular
  long dt; //delta de tiempo para calcular desplazamiento angular
  if (tiempoPrev == 0) {
    tiempoPrev = micros();
  }
  segundoI2C.beginTransmission(0x68);   //empieza a comunicar con el mpu6050
  segundoI2C.write(0x3B);   //envia byte 0x43 al sensor para indicar startregister
  segundoI2C.endTransmission();   //termina comunicacion
  segundoI2C.requestFrom(0x68, 14); //pide 6 bytes al sensor, empezando del reg 43 (ahi estan los valores del giro)

  if (14 <= segundoI2C.available()) {
    ac_x = segundoI2C.read() << 8 | segundoI2C.read();
    ac_y = segundoI2C.read() << 8 | segundoI2C.read();
    ac_z = segundoI2C.read() << 8 | segundoI2C.read();
  
    tmp = segundoI2C.read() << 8 | segundoI2C.read();
  
    gyro_x = segundoI2C.read() << 8 | segundoI2C.read(); //combina los valores del registro 44 y 43, desplaza lo del 43 al principio
    gyro_y = segundoI2C.read() << 8 | segundoI2C.read();
    gyro_z = segundoI2C.read() << 8 | segundoI2C.read();
  }
  
  unsigned long tiempoYaMicros= micros();
  dt =  tiempoYaMicros - tiempoPrev;
  tiempoPrev = tiempoYaMicros;

  //Divide los datos entre sus ganancias
  ay = ((float(ac_y) - acy_off) / A_R) * (9.81);
  gz = (float(gyro_z) - gz_off) / G_R;

  //Filtro, aun no se está usando, si se desear usar, cambiar 1 por 0.05
  ayft = ay * 1 + (1 - 1) * ayft;
  gzft = gz * 1 + (1 - 1) * gzft;

  gir_ang_z = gzft * (dt / 1000000.0) + gir_ang_zPrev;
  vely = ayft * (dt / 1000000.0);

  gir_ang_zPrev = gir_ang_z;
  vel_y_Prev = vely;
}






//*****Cementerio de funciones. Borrar en Junio 2021********


//void RevisarSTM(){ 
////Comunica al STM mediante pin de interrupcion que esta listo para recibir información de obstaculos 
//  if((micros()-tiempoSTM)>=tiempoMuestreo/4){ //Atender obstaculos 4 veces por cada ciclo de la maquina de estados
//    tiempoSTM=micros();
//    if(estado!=RETROCEDA){ //Ignorar al STM si está retrocediendo
//      digitalWrite(INT_OBSTACULO,HIGH);
//      //delay(5);
//      digitalWrite(INT_OBSTACULO,LOW);
//      //delay(5);
//    }
//  }
//}

/*
void ActualizarUbicacion() {
  //Función que actualiza la ubicación actual en base al avance anterior y la orientación actual
  distanciaAvanzada = (int)calculaDistanciaLinealRecorrida();
  if (poseActual[2] == 0 || abs(poseActual[2] == 360)) {
    poseActual[1] = poseActual[1] + distanciaAvanzada;
  }
  else if (poseActual[2] == 90 || poseActual[2] == -270) {
    poseActual[0] = poseActual[0] + distanciaAvanzada;
  }
  else if (poseActual[2] == -90 || poseActual[2] == 270) {
    poseActual[0] = poseActual[0] - distanciaAvanzada;
  }
  else if (abs(poseActual[2]) == 180) {
    poseActual[1] = poseActual[1] - distanciaAvanzada;
  }
  
}
*/

//void RecibirI2C ()  {
//  //Función (tipo Evento) llamada cuando se recibe algo en el puerto I2C conectado al STM32
//  //Almacena en una matriz las variables de tipo de sensor, distancia y ángulo
//  int cont = 1;
//  int tipoSensor = 0;
//  int distancia = 0;
//  int angulo = 0;
//  String acumulado = "";
//  DeteccionObstaculo();
//
//  while (0 < Wire.available()) { // ciclo mientras se reciben todos los datos
//    char c = Wire.read(); // se recibe un byte a la vez y se maneja como char
//    if (c == ',') { //los datos vienen separados por coma
//      if (cont == 1) {
//        tipoSensor = acumulado.toInt();
//        acumulado = "";
//      }
//      if (cont == 2) {
//        distancia = acumulado.toInt();
//        acumulado = "";
//      }
//      cont++;
//    } else if (c == '.') { //el ultimo dato viene con punto al final
//      angulo = acumulado.toInt();
//      acumulado = "";
//      cont = 1;
//    } else {
//      acumulado += c;  //añade el caracter a la cadena anterior
//    }
//  }
//
//  float alphaObs = (float)(medirMagnet() + angulo);
//  if(alphaObs > 180){ //Correción para tener valores entre -180 y 180
//    alphaObs = alphaObs - 360;          
//  }
//  else if(alphaObs < -180){
//    alphaObs = alphaObs + 360;
//  }
//
//  alphaObs = alphaObs - magInicioOrigen;
//  if(alphaObs > 180){ //Correción para tener valores entre -180 y 180
//    alphaObs = alphaObs - 360;          
//  }
//  else if(alphaObs < -180){
//    alphaObs = alphaObs + 360;
//  }
//
//  CrearObstaculo(tipoSensor, distancia, alphaObs);
//
//  //obstaculoEnviado = false;
//  //CrearMensaje(poseActual[0], poseActual[1], poseActual[2], tipoSensor, distancia, alphaObs);
//  
//}
