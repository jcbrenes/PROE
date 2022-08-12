// rf69 demo tx rx.pde
// -*- mode: C++ -*-
// Archivo para la experimentación del objetivo 2 del proyecto final de graduación
// Confirme que en el serial de arduino se tenga "no line ending" para enviar datos en el serial

#include <SPI.h>
#include <RH_RF69.h>
#include <RHDatagram.h>

/************ Serial Setup ***************/
#define debug 1

#if debug == 1
#define serialPrint(x) Serial.print(x)
#define serialPrintln(x) Serial.println(x)
#else
#define serialPrint(x)
#define serialPrintln(x)
#endif

/************ Radio Setup ***************/

// Change to 434.0 or other frequency, must match RX's freq!
#define RF69_FREQ 915.0

// change addresses for each client board,
#define MY_ADDRESS     0

#define RFM69_CS      8
#define RFM69_INT     3
#define RFM69_RST     4
#define LED           13

// Singleton instance of the radio driver
RH_RF69 rf69(RFM69_CS, RFM69_INT);

// Class to manage message delivery and receipt, using the driver declared above
RHDatagram rf69_manager(rf69, MY_ADDRESS);

//Variables para el mensaje que se va a transmitir a la base
bool timeReceived = false; //Bandera para saber si ya recibí el clock del máster.
unsigned long tiempoRobotTDMA = 20; //Slot de tiempo que tiene cada robot para hablar. Unidades ms.
unsigned long tiempoCicloTDMA = 20; //Duración de todo un ciclo de comunicación TDMA. Unidades ms.
unsigned long timeStamp1; //Estampa de tiempo para eliminar el desfase por procesamiento del reloj al recibirse. Se obtiene apenas se recibe el reloj.

int cantidadRobots = 2; // Cantidad total de robots en el enjambre
int unidadAvance = 485; // Medida en mm que avanza cada robot por movimiento

// Dont put this on the stack:
uint8_t buf[RH_RF69_MAX_MESSAGE_LEN];
uint8_t len = sizeof(buf);
uint8_t from;

int16_t posX;
int16_t posY;
int16_t rot;
int8_t tipSens;
int16_t dis;
int16_t angulo;

// Input para leer los datos del serial
char input;

void setup()
{
  if (debug == 1){
    Serial.begin(19200);
    while (!Serial); // wait until serial console is open, remove if not tethered to computer 
  }

  serialPrintln("Prueba base");
  serialPrintln();
  
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
  RTC->MODE0.COMP[0].reg = 200UL;       //Valor inicial solo por poner un valor. Más adelante, apenas reciba el clock del master, ya actualiza este valor correctamente. ¿Por qué debo agregar el [0]? Esto nunca lo entendí.
  while (RTCisSyncing());               //Llamo a función de escritura.

  // RTC->MODE0.INTENSET.bit.CMP0 = true;  //Habilito la interrupción.
  // RTC->MODE0.INTFLAG.bit.CMP0 = true;   //Remover la bandera de interrupción.

  // NVIC_EnableIRQ(RTC_IRQn);             //Habilito la interrupción del RTC en el "Nested Vestor
  // NVIC_SetPriority(RTC_IRQn, 0x00);     //Seteo la prioridad de la interrupción. Esto se debe investigar más.
  // RTCenable();                          //Habilito de nuevo el RTC.
  // RTCresetRemove();                     //Quito el reset del RTC.

  serialPrintln("¡RFM69 radio en funcionamiento!");
  delay(20);

  //sendUnidadAvance(&unidadAvance); // Envía la unidad de avance en mm a utilizar por los robots
  
  //sendCantidadRobots(&cantidadRobots); // Envía la cantidad de robots en el enjambre

  sincronizar(); // Envía el valor de clock a los robots del enjambre
  serialPrintln("time;estado;ID;posX;posY;rot;tipSens;dis;angulo");
}

void loop() {
  // Código
  if (rf69_manager.available()){
    if (rf69_manager.recvfrom(buf, &len, &from)){
      buf[len] = 0;
      posX = *(int16_t*)&buf[0];
      posY = *(int16_t*)&buf[2];
      rot = *(int16_t*)&buf[4];
      tipSens = (int8_t)buf[6];
      dis = *(int16_t*)&buf[7];
      angulo = *(int16_t*)&buf[9];
    }
    runningTime(RTC->MODE0.COUNT.reg);serialPrint("recibido;");
    serialPrint(from);serialPrint(";");serialPrint(posX);serialPrint(";");serialPrint(posY);
    serialPrint(";");serialPrint(rot);serialPrint(";");serialPrint(tipSens);
    serialPrint(";");serialPrint(dis);serialPrint(";");serialPrintln(angulo);

    //serialPrint("[RSSI ");serialPrint(rf69.lastRssi());serialPrintln("]");
    
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

/**** RTC FUNCIONES****/

/// \param cantRobots Puntero hacía la variable con la cantidad de robots en el enjambre
/// \return Devuelve true si se envía el mensaje y false en caso contrario
bool sendCantidadRobots(int *cantRobots){
  uint8_t dataCantRobots[sizeof(int)];
  uint32_t* ptrCantRobots = (uint32_t*)cantRobots;
  for(uint32_t i = 0; i < 4; i++){
    dataCantRobots[i] = *ptrCantRobots >> i*8;  //La parte de "(255UL << i*8)) >> i*8" es solo para ir acomodando los bytes en el array de envío mensaje[].
  }
  return rf69_manager.sendto(dataCantRobots, sizeof(dataCantRobots), RH_BROADCAST_ADDRESS);
}

/// \param Avance Unidad de avance para los robots
/// \return Devuelve true si se envía el mensaje y false en caso contrario
bool sendUnidadAvance(int *Avance){
  uint8_t dataUnidadAvance[sizeof(int)];
  uint32_t* ptrdataUnidadAvance = (uint32_t*)Avance;
  for(uint32_t i = 0; i < 4; i++){
    dataUnidadAvance[i] = *ptrdataUnidadAvance >> i*8;  //La parte de "(255UL << i*8)) >> i*8" es solo para ir acomodando los bytes en el array de envío mensaje[].
  }
  return rf69_manager.sendto(dataUnidadAvance, sizeof(dataUnidadAvance), RH_BROADCAST_ADDRESS);
}

/// \brief Envía el valor del clock en el registro para sincronizar los robots.
/// \return Devuelve true si se envía el mensaje y false en caso contrario.
bool sincronizar(){
  timeStamp1 = RTC->MODE0.COUNT.reg; //Extraer tiempo del RTC de la base
  uint8_t reloj[4];
  uint32_t* ptrReloj = (uint32_t*)&timeStamp1;       //Utilizo el puntero para extraer la información del dato flotante.
  for(uint8_t i = 0; i < 4; i++){
    reloj[i] = *ptrReloj >> i*8;  //La parte de "(255UL << i*8)) >> i*8" es solo para ir acomodando los bytes en el array de envío mensaje[].
  }

  return rf69_manager.sendto(reloj, sizeof(reloj), RH_BROADCAST_ADDRESS);     //Enviar valor del RTC al esclavo
}

/**** RTC FUNCIONES****/

inline bool RTCisSyncing() {
  //Función que lee el bit de sincronización de los registros
  return (RTC->MODE0.STATUS.bit.SYNCBUSY);
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

    serialPrintln("Activación de interrupción");
    runningTime(RTC->MODE0.COUNT.reg);

    RTC->MODE0.INTFLAG.bit.CMP0 = true;         //Limpiar la bandera de la interrupción.
  }
}