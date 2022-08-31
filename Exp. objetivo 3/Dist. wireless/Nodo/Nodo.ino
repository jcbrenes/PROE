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

//int timeExperimentMinutes = 1; // Tiempo del experimento en minutos
// 1 minu = 60_000
int timeExperiment = 18000; //timeExperimentMinutes * 6000;

//Variables del enjambre para la comunicación a la base
uint8_t cantidadRobots = 2; //Cantidad de robots en enjambre. No cuenta la base, solo los que hablan.
unsigned long idRobot = 1; //ID del robot, este se usa para ubicar al robot dentro de todo el ciclo de TDMA.

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
uint8_t from;
uint16_t* ptrMensaje; //Puntero para descomponer datos en bytes.
uint8_t mensaje[5 * sizeof(uint16_t) + sizeof(uint8_t)]; // Mensaje a enviar a la base

void setup() {
  // Se asigna la semilla aleatoria 
  randomSeed(analogRead(A5));

  //***Inicialización de puertos seriales***
  if (debug == 1){
    Serial.begin(19200); //Puerto serie para comunicación con la PC
    while (!Serial);
  }
  
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

  rf69.setTxPower(13, true); // Configura la potencia puede ser de -2 a 20
  // Se coloca en 13 para configurarlo como baja potencia

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

  serialPrintln("¡RFM69 radio en funcionamiento!");
  delay(20);
  serialPrint("Robot: ");serialPrintln(idRobot);
  //******En caso de usar el robot solo (no como enjambre), comentar la siguiente linea
  sincronizacion(); //Esperar mensaje de sincronizacion de la base antes de moverse
  serialPrintln("time;estado;ID;posX;posY;rot;tipSens;dis;angulo");
  //descomentar la siguiente linea si se quiere llevar el robot a cierta coordenada
  //estado = CONTROL_POSE;
    
}

// Generar números aleatorios
// 16 son 32000
// 8 son 125
int posX = 1;
int posY;
int rot;
int tipSens;
int dis;
int angulo;

// Stop process
bool timeReady = false;

void loop(){
  if (RTC->MODE0.COUNT.reg > timeExperiment){
    timeReady = true;
    Serial.end();
  }
  if (!timeReady){
    if (!mensajeCreado){
      posY = random(-32000,32000);
      rot = random(-32000,32000);
      tipSens = random(-10,10);
      dis = random(-32000,32000);
      angulo = random(-32000,32000);
      CrearMensaje(posX, posY, rot, tipSens, dis, angulo);
      posX += 1;
    }
    if (rf69_manager.available()){
      if (rf69_manager.recvfrom(buf, &len, &from)){
        buf[len] = 0;
        runningTime(RTC->MODE0.COUNT.reg);serialPrint("recibido;");
        serialPrint(from);serialPrint(";");serialPrint(*(int16_t*)&buf[0]);serialPrint(";");serialPrint(*(int16_t*)&buf[2]);
        serialPrint(";");serialPrint(*(int16_t*)&buf[4]);serialPrint(";");serialPrint((int8_t)buf[6]);
        serialPrint(";");serialPrint(*(int16_t*)&buf[7]);serialPrint(";");serialPrintln(*(int16_t*)&buf[9]);
      }
    }
  }
}

/// \param regMili Valor del counter del RTC
/// \return Imprime el tiempo que ha corrido el programa
void runningTime(unsigned long regMili){
  int miliS = regMili % 1000;
  regMili /= 1000;
  int secS = regMili % 60;
  regMili /= 60;
  int minS = regMili % 60;
  serialPrint(minS);serialPrint(":");serialPrint(secS);serialPrint(".");serialPrint(miliS);serialPrint(";");
}

/// \brief Crea el mensaje a ser enviado 
/// \param poseX Posición X del robot
/// \param poseY Posición Y del robot
/// \param rotacion Ángulo del robot
/// \param tipoSensorX Obtáculo encontrado
/// \param distanciaX Distancia del obstáculo
/// \param anguloX Ángulo detección obstáculo
void CrearMensaje(int poseX, int poseY, int rotacion, int tipoSensorX, int distanciaX, int anguloX){ 
//Crea el mensaje a ser enviado por comunicación RF. Es usado en la interrupcion del RTC
  
  if(timeReceived && !mensajeCreado){       //Aquí solo debe enviar mensajes, pero eso lo hace la interrupción, así que aquí se construyen mensajes y se espera a que la interrupción los envíe.

    //Genero números al azar acorde a lo que el robot eventualmente podría enviar
    uint16_t xP = (uint16_t)poseX;                //Posición X en que se detecto el obstaculo
    uint16_t yP = (uint16_t)poseY;                //Posición Y en que se detecto el obstaculo
    uint16_t phi = (uint16_t)rotacion;            //"Orientación" del robot
    uint8_t tipo = (uint8_t)tipoSensorX;        //Tipo de sensor que se detecto (Sharp, IR Fron, IR Der, IR Izq, Temp, Bat)
    uint16_t rObs = (uint16_t)distanciaX;         //Distancia medida por el sharp si aplica
    uint16_t alphaObs = (uint16_t)(anguloX);      //Angulo respecto al "norte" (orientación inicial del robot medida por el magnetometro)

    runningTime(RTC->MODE0.COUNT.reg);serialPrint("enviado;");serialPrint(rf69_manager.thisAddress());serialPrint(";");serialPrint(poseX);serialPrint(";");serialPrint(poseY);
    serialPrint(";");serialPrint(rotacion);serialPrint(";");serialPrint(tipoSensorX);
    serialPrint(";");serialPrint(distanciaX);serialPrint(";");serialPrintln(anguloX);

    //Construyo mensaje (es una construcción bastante manual que podría mejorar)
    ptrMensaje = (uint16_t*)&xP;       //Utilizo el puntero para extraer la información del dato flotante.
    for (uint8_t i = 0; i < 2; i++) {
      mensaje[i] = (*ptrMensaje & (255UL << i * 8)) >> i * 8; //La parte de "(255UL << i*8)) >> i*8" es solo para ir acomodando los bytes en el array de envío mensaje[].
    }

    ptrMensaje = (uint16_t*)&yP;
    for (uint8_t i = 2; i < 4; i++) {
      mensaje[i] = (*ptrMensaje & (255UL << (i - 2) * 8)) >> (i - 2) * 8;
    }

    ptrMensaje = (uint16_t*)&phi;
    for (uint8_t i = 4; i < 6; i++) {
      mensaje[i] = (*ptrMensaje & (255UL << (i - 4) * 8)) >> (i - 4) * 8;
    }

    ptrMensaje = (uint16_t*)&tipo;
    for (uint8_t i = 6; i < 7; i++) {
      mensaje[i] = (*ptrMensaje & (255UL << (i - 6) * 8)) >> (i - 6) * 8;
    }

    ptrMensaje = (uint16_t*)&rObs;
    for (uint8_t i = 7; i < 9; i++) {
      mensaje[i] = (*ptrMensaje & (255UL << (i - 7) * 8)) >> (i - 7) * 8;
    }

    ptrMensaje = (uint16_t*)&alphaObs;
    for (uint8_t i = 9; i < 11; i++) {
      mensaje[i] = (*ptrMensaje & (255UL << (i - 9) * 8)) >> (i - 9) * 8;
    }

    //Una vez creado el mensaje, no vuelvo a crear otro hasta que la interrupción baje la bandera.
    mensajeCreado = true;
  }
}


/**** RTC FUNCIONES****/

inline bool RTCisSyncing() {
  //Función que lee el bit de sincronización de los registros
  return (RTC->MODE0.STATUS.bit.SYNCBUSY);
}

void sincronizacion() {
  while (!timeReceived) { //Espera mensaje de sincronización de la base

    if (rf69.available()){                                                        // Espera a recibir un mensaje
      uint8_t len = sizeof(buf);                                                  //Obtengo la longitud máxima del mensaje a recibir
      timeStamp1 = RTC->MODE0.COUNT.reg;
      if (rf69.recv(buf, &len)) {                                     //Llamo a recv() si recibí algo. El timeStamp1 guarda el valor del RTC del nodo en el momento en que comienza a procesar el mensaje.
        buf[len] = 0;                                                             //Limpio el resto del buffer.
        data = buf[3] << 24 | buf[2] << 16 | buf[1] << 8 | buf[0];                //Construyo el dato con base en el buffer y haciendo corrimiento de bits.
        timeStamp2 = RTC->MODE0.COUNT.reg;                                        //Una vez procesado el mensaje del reloj del máster, guardo otra estampa de tiempo para eliminar este tiempo de procesamiento.
        unsigned long masterClock = data + (timeStamp2 - timeStamp1) + timeOffset; //Calculo reloj del master como: lo enviado por el máster (data) + lo que dura el mensaje en llegarme (timeOffset) + lo que duré procesando el mensaje (tS2-tS1).
        RTC->MODE0.COUNT.reg = masterClock;                                       //Seteo el RTC del nodo al tiempo del master.
        RTC->MODE0.COMP[0].reg = masterClock + idRobot * tiempoRobotTDMA + 50;    //Partiendo del clock del master, calculo la próxima vez que tengo que hacer la interrupción según el ID. Sumo 50 ms para dejar un colchón que permita que todos los robots oigan el mensaje antes de empezar a hablar.
        while (RTCisSyncing());                                                   //Espero la sincronización.

        serialPrintln("¡Reloj del máster clock recibido!");
        timeReceived = true;                                                      //Levanto la bandera que indica que recibí el reloj del máster y ya puedo pasar a transmitir.
        //Wire.onReceive(RecibirI2C);
      }
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
  if (RTC->MODE0.COUNT.reg > 0x00000064 && timeReady == false) {      //Si el valor del contador es mayor a 0x64 (100 en decimal) entonces haga la interrupción. Esto para evitar el problema que la interrupción se llame al puro inicio. No sé por qué pasaba esto, investigar más el tema.
    RTC->MODE0.COMP[0].reg += tiempoCicloTDMA;  //Quiero que haga una interrupción en el próximos ciclo del TDMA, actualizo el nuevo valor a comparar.  **¿Por qué debo agregar el [0]?**
    while (RTCisSyncing());                     //Llamo a función de escritura.

    rf69_manager.sendto(mensaje, sizeof(mensaje), RH_BROADCAST_ADDRESS);        //Llamo a la función de enviar de la biblioteca Radio Head para enviar el mensaje del nodo.
    mensajeCreado = false;                      //Bajo la bandera para indicar que ya se envió el mensaje y se puede crear uno nuevo.

    RTC->MODE0.INTFLAG.bit.CMP0 = true;         //Limpiar la bandera de la interrupción.
  }
}
