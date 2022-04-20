// Prueba de sincronización de reloj
// Se cambia el tiempo del clock a 1kHz

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


//**** Variables para las pruebas en el laboratorio ****//
uint8_t cantidadRobots; // Cantidad de robots en el enjambre 
int unidadAvance; // Unidad en mm que avance cada robots del enjambre
bool dataRecieved = false; // Se comprueba que se haya recibido el dato

/************ Radio Setup ***************/

//Variables para el mensaje que se va a transmitir a la base
bool timeReceived = false; //Bandera para saber si ya recibí el clock del máster.
unsigned long tiempoRobotTDMA = 50; //Slot de tiempo que tiene cada robot para hablar. Unidades ms.
unsigned long tiempoCicloTDMA = 10 * tiempoRobotTDMA; //Duración de todo un ciclo de comunicación TDMA. Unidades ms.
unsigned long timeStamp1; //Estampa de tiempo para eliminar el desfase por procesamiento del reloj al recibirse. Se obtiene apenas se recibe el reloj.
unsigned long timeStamp2;
const uint8_t timeOffset = 2;
unsigned long data; //Variable donde se almacenará el valor del reloj del máster.
unsigned long tiempo;
uint8_t buf[RH_RF69_MAX_MESSAGE_LEN];
uint8_t len = sizeof(buf);

// Frecuencia a la que se va a trabajar con el RF69
#define RF69_FREQ 915.0

// Dirección del robot
#define MY_ADDRESS     8

#define RFM69_CS      8
#define RFM69_INT     3
#define RFM69_RST     4
#define LED           13

// Se declara la instancia del driver
RH_RF69 rf69(RFM69_CS, RFM69_INT);

// Class to manage message delivery and receipt, using the driver declared above
RHDatagram rf69_manager(rf69, MY_ADDRESS);

/*********** Setup ***********/

void setup()
{
  if (debug == 1){
    Serial.begin(9600);
    while (!Serial); // Se espera a que la comunicación serial inicie, si no se va a usar debug = 0
  }
  
  pinMode(LED, OUTPUT);
  pinMode(RFM69_RST, OUTPUT);
  digitalWrite(RFM69_RST, LOW);

  serialPrintln("Prueba del nodo");
  serialPrintln();

  // manual reset
  digitalWrite(RFM69_RST, HIGH);
  delay(10);
  digitalWrite(RFM69_RST, LOW);
  delay(10);
  
  if (!rf69_manager.init()) {
    serialPrintln("RFM69 radio init failed");
    while (1);
  }
  serialPrintln("RFM69 radio init OK!");
  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM (for low power module)
  // No encryption
  if (!rf69.setFrequency(RF69_FREQ)) {
    serialPrintln("setFrequency failed");
  }

  // If you are using a high power RF69 eg RFM69HW, you *must* set a Tx power with the
  // ishighpowermodule flag set like this:
  rf69.setTxPower(20, true);  // range from 14-20 for power, 2nd arg must be true for 69HCW
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
  
  pinMode(LED, OUTPUT);

  serialPrint("RFM69 radio @");  serialPrint((int)RF69_FREQ);  serialPrintln(" MHz");
  
  serialPrintln("Esperando unidad de avance...");
  unidadAvance = waitUnidadAvance(&dataRecieved);
  serialPrint("Unidad de avance: "); serialPrintln(unidadAvance);
  dataRecieved = false;

  serialPrintln("Esperando cantidad de robots...");
  cantidadRobots = waitCantidadRobots(&dataRecieved);
  serialPrint("Cantidad de robots recibido: "); serialPrintln(cantidadRobots);
  dataRecieved = false;
}

void loop() {
    sincronizacion();
}

/**** RTC FUNCIONES ****/

inline bool RTCisSyncing() {
  //Función que lee el bit de sincronización de los registros
  return (RTC->MODE0.STATUS.bit.SYNCBUSY);
}

void configureClock() {
  //Función que configura el clock. Se debe estudiar más a detalle los comandos.
  GCLK->GENDIV.reg = GCLK_GENDIV_ID(2) | GCLK_GENDIV_DIV(4);
  while (GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY);
  
  GCLK->GENCTRL.reg = (GCLK_GENCTRL_GENEN | GCLK_GENCTRL_SRC_XOSC32K | GCLK_GENCTRL_ID(2) | GCLK_GENCTRL_DIVSEL );
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

    timeStamp1 = RTC->MODE0.COUNT.reg;
    serialPrintln(timeStamp1);
    
    RTC->MODE0.INTFLAG.bit.CMP0 = true;         //Limpiar la bandera de la interrupción.
  }
}

/// \fn void sincronizacion()
/// \brief Espera que se realice la sincronización del clock
void sincronizacion() {
  while (!timeReceived) { //Espera mensaje de sincronización de la base
    if (rf69_manager.available()){
      timeStamp1 = RTC->MODE0.COUNT.reg;
      if (rf69_manager.recvfrom(buf, &len)) {                                     //Llamo a recv() si recibí algo. El timeStamp1 guarda el valor del RTC del nodo en el momento en que comienza a procesar el mensaje.
        buf[len] = 0;                                                             //Limpio el resto del buffer.
        data = buf[3] << 24 | buf[2] << 16 | buf[1] << 8 | buf[0];                //Construyo el dato con base en el buffer y haciendo corrimiento de bits.
        timeStamp2 = RTC->MODE0.COUNT.reg;                                        //Una vez procesado el mensaje del reloj del máster, guardo otra estampa de tiempo para eliminar este tiempo de procesamiento.
        unsigned long masterClock = data + (timeStamp2 - timeStamp1) + timeOffset; //Calculo reloj del master como: lo enviado por el máster (data) + lo que dura el mensaje en llegarme (timeOffset) + lo que duré procesando el mensaje (tS2-tS1).
        RTC->MODE0.COUNT.reg = masterClock;                                       //Seteo el RTC del nodo al tiempo del master.
        RTC->MODE0.COMP[0].reg = masterClock + 2 * tiempoRobotTDMA + 50;    //Partiendo del clock del master, calculo la próxima vez que tengo que hacer la interrupción según el ID. Sumo 50 ms para dejar un colchón que permita que todos los robots oigan el mensaje antes de empezar a hablar.
        
        while (RTCisSyncing());                                                   //Espero la sincronización
        timeReceived = true;                                                      //Levanto la bandera que indica que recibí el reloj del máster y ya puedo pasar a transmitir.
        serialPrintln("¡Reloj del máster clock recibido!");
      }
    }
  }
}

/// \fn int waitUnidadAvance()
/// \param messageRecieved Puntero variable recibido unidad de avance
/// \brief Espera recibir la unidad de avance a utilizar en los robots
/// \return Devuelve el valor de la unidad de avance
int waitUnidadAvance(bool *messageRecieved){
    while (!*messageRecieved){
        if (rf69_manager.available()){
            if (rf69_manager.recvfrom(buf, &len)) {
              buf[len] = 0;
              data = buf[3] << 24 | buf[2] << 16 | buf[1] << 8 | buf[0];
              *messageRecieved = true;
              serialPrintln("Recibido: unidad avance");
            }
        }
    }
    return (int)data;    
}

/// \fn int waitCantidadRobots()
/// \param messageRecieved Puntero variable recibido unidad de avance
/// \brief Espera recibir la cantidad de robots a utilizar en el enjambre
/// \return Devuelve la cantidad de robots en el enjambre
uint8_t waitCantidadRobots(bool *messageRecieved){
    while (!*messageRecieved){
        if (rf69_manager.available()){
            if (rf69_manager.recvfrom(buf, &len)) {
                buf[len] = 0;
                data = buf[0];
                *messageRecieved = true;
                serialPrintln("Recibido: cantidad robots");
            }
        }
    }
    return (uint8_t)data;
}
