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

int16_t distCoordenadas; // Recibe la distancia entre los robots al inicio de la configuración

// Input para leer los datos del serial
char input;

void setup()
{
  if (debug == 1){
    Serial.begin(115200);
    while (!Serial); // wait until serial console is open, remove if not tethered to computer 
  }

  //serialPrintln("Prueba base");
  //serialPrintln();
  
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

  //serialPrintln("¡RFM69 radio en funcionamiento!");
  delay(20);

  //sendUnidadAvance(&unidadAvance); // Envía la unidad de avance en mm a utilizar por los robots
  
  //sendCantidadRobots(&cantidadRobots); // Envía la cantidad de robots en el enjambre

  sincronizar(); // Envía el valor de clock a los robots del enjambre
}

void loop() {
  // Código
  if (rf69_manager.available()){
    if (rf69_manager.recvfrom(buf, &len, &from)){
      buf[len] = 0;
      distCoordenadas = *(int16_t*)&buf[0];
    }
    serialPrint(from);serialPrint(' ');serialPrintln(distCoordenadas);
  }
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