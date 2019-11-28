#include <SPI.h>
#include <RH_RF69.h>

/************ Radio Setup ***************/
#define RF69_FREQ      915.0  //La frecuencia debe ser la misma que la de los demas nodos.
#define DEST_ADDRESS   10     //No sé si esto es totalmente necesario, creo que no porque nunca usé direcciones.
#define MY_ADDRESS     4      //Dirección de este nodo. Creo que no tampoco es necesario.
// Definición de pines. Creo que no todos se están usando, me parece que el LED no.
#define RFM69_CS       8
#define RFM69_INT      3
#define RFM69_RST      4
#define LED           13

// Inicialización del driver de la biblioteca Radio Head.
RH_RF69 rf69(RFM69_CS, RFM69_INT);

uint8_t buf[RH_RF69_MAX_MESSAGE_LEN];                            //Buffer para recibir mensajes.
bool timeRecieved = false;                                       //Bandera para saber si ya recibí el clock del máster.
unsigned long tiempoRobotTDMA = 50;                              //Slot de tiempo que tiene cada robot para hablar. Unidades ms.
uint8_t cantidadRobots = 4;                                      //Cantidad de robots en enjambre. No cuenta la base, solo los que hablan.
unsigned long tiempoCicloTDMA = cantidadRobots*tiempoRobotTDMA;  //Duración de todo un ciclo de comunicación TDMA. Unidades ms.
unsigned long idRobot = 4;                                       //ID del robot, este se usa para ubicar al robot dentro de todo el ciclo de TDMA.
unsigned long timeStamp1;                                        //Estampa de tiempo para eliminar el desfase por procesamiento del reloj al recibirse. Se obtiene apenas se recibe el reloj.
unsigned long timeStamp2;                                        //Estampa de tiempo para eliminar el desfase por procesamiento del reloj al recibirse. Se obtiene al guardar en memoria el reloj.
unsigned long data;                                              //Variable donde se almacenará el valor del reloj del máster.
volatile bool mensajeCreado = false;                             //Bandera booleana para saber si ya debo crear otro mensaje cuando esté en modo transmisor.
uint8_t mensaje[7*sizeof(float)];                                //El mensaje a enviar. Llevará 6 datos tipo float: (robotID, xP, yP, phi, tipoObs, rObs, alphaObst)
uint32_t* ptrMensaje;                                            //Puntero para descomponer el float en bytes.
const uint8_t timeOffset = 2;                                    //Offset que existe entre el máster enviando y el nodo recibiendo.
        //El 2 de arriba es una constante para que cierre el cálculo con el líder.
        //En teoría, estos 2 ms son el tiempo que dura el transmisor enviando por 
        //spi su RTC, activando su transmisor y transmitiendo. Además del tiempo
        //que dura el receptor dándose cuenta que recibió algo. El timeStamp1-timeStamp2
        //compensan el tiempo de procesamiento en el receptor y el tiempo que haya
        //pasado entre el print y el recibimiento del clock del master.

void setup() {
  /****General****/
  Serial.begin(9600);
  pinMode(LED, OUTPUT);     
  pinMode(RFM69_RST, OUTPUT);
  digitalWrite(RFM69_RST, LOW);

  /****RF****/
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
  // Configurar potencia
  rf69.setTxPower(20, true);  //Rango de 14-20 para la potencia, segundo argumento debe ser verdadero para el 69HCW.
  // Configurar aL NODO para que escuche cualquier dirección (esto sí es necesario).
  rf69.setPromiscuous(true);
  //Empezar como receptor para estar listos al mensaje del máster.
  rf69.setModeRx();

  /****RTC****/


  //Para entender esto y las funciones que se llaman (están al final del código) es necesario estudiar el capítulo 19 del datasheet del microcontrolador, es sobre el periférico RTC.
  //Para entender la referencia de punteros, ir a: 
  //C:\Users\Carlos Barrantes\AppData\Local\Arduino15\packages\arduino\hardware\samd\1.6.19\bootloaders\mzero\Bootloader_D21\src\ASF\sam0\utils\cmsis\samd21\include\component
  //Al menos en mi computadora está ahí, corregir Usuario. Ahí están como los cógidos de construcción de estructuras de programación de periféricos, ahí se pueden ver mejor
  //las relaciones entre los nombres del datasheet, el uso de punteros y la manera de usarlos en programación.
  //Links de ayuda:
  //1. Acceso a los registros del SAM MCU en C: https://microchipdeveloper.com/32arm:sam-bare-metal-c-programming#toc3
  //2. Configuración de interrupciones del SAM MCU en C: https://microchipdeveloper.com/32arm:samd21-nvic-configuration
  //Para las interrupciones también es necesario estudiar algo que se llama "Nested Vector Interrupt Controller", que es propio del SAMD21.

  
  PM->APBAMASK.reg |= PM_APBAMASK_RTC;  //Seteo la potencia para el reloj del RTC. (Este paso hay que estudiarlo más, pues solo lo copie de una biblioteca ya hecha)
  //config32kOSC();                     //Esta función hay que estudiarla más, es para setear el reloj externo, no la usé.
  configureClock();                     //Seteo reloj interno de 32kHz. Esta función hay que estudiarla más, es para setear el reloj interno.
  RTCdisable();                         //Deshabilito RTC. Hay configuraciones que solo se pueden hacer si el RTC está deshabilitado, revisar datasheet.
  RTCreset();                           //Reseteo RTC.
  
  RTC->MODE0.CTRL.reg = 2UL;            //Esto en binario es 0000000000000010. Esa es la configuración que ocupo en el registro. Ver capítulo 19 del datasheet.
  while(RTCisSyncing());                //Llamo a función de escritura. Hay bits que deben ser sincronizados cada vez que se escribe en ellos. Esto lo hice por precaución..
  RTC->MODE0.COUNT.reg = 0UL;           //Seteo el contador en 0 para iniciar.
  while(RTCisSyncing());                //Llamo a función de escritura.
  RTC->MODE0.COMP[0].reg = 600000UL;    //Valor inicial solo por poner un valor. Más adelante, apenas reciba el clock del master, ya actualiza este valor correctamente. ¿Por qué debo agregar el [0]? Esto nunca lo entendí.
  while(RTCisSyncing());                //Llamo a función de escritura.
    
  RTC->MODE0.INTENSET.bit.CMP0 = true;  //Habilito la interrupción.
  RTC->MODE0.INTFLAG.bit.CMP0 = true;   //Remover la bandera de interrupción.

  NVIC_EnableIRQ(RTC_IRQn);             //Habilito la interrupción del RTC en el "Nested Vestor  
  NVIC_SetPriority(RTC_IRQn, 0x00);     //Seteo la prioridad de la interrupción. Esto se debe investigar más.

  RTCenable();                          //Habilito de nuevo el RTC.
  RTCresetRemove();                     //Quito el reset del RTC.
  
  Serial.println("¡RFM69 radio OK!");
}

void loop() {

  //Loop para escuchar el clock del máster.
  while(!timeRecieved){
      uint8_t len = sizeof(buf);                                                  //Obtengo la longitud máxima del mensaje a recibir
      if(rf69.recv(buf, &len, timeStamp1)){                                       //Llamo a recv() si recibí algo. El timeStamp1 guarda el valor del RTC del nodo en el momento en que comienza a procesar el mensaje.
        Serial.println("¡Reloj del máster clock recibido!");
        buf[len] = 0;                                                             //Limpio el resto del buffer.
        data = buf[3] << 24 | buf[2] << 16 | buf[1] << 8 | buf[0];                //Construyo el dato con base en el buffer y haciendo corrimiento de bits.
        timeStamp2 = RTC->MODE0.COUNT.reg;                                        //Una vez procesado el mensaje del reloj del máster, guardo otra estampa de tiempo para eliminar este tiempo de procesamiento. 
        unsigned long masterClock = data + (timeStamp2-timeStamp1) + timeOffset;  //Calculo reloj del master como: lo enviado por el máster (data) + lo que dura el mensaje en llegarme (timeOffset) + lo que duré procesando el mensaje (tS2-tS1).
        RTC->MODE0.COUNT.reg = masterClock;                                       //Seteo el RTC del nodo al tiempo del master.     
        RTC->MODE0.COMP[0].reg = masterClock + idRobot*tiempoRobotTDMA + 50;      //Partiendo del clock del master, calculo la próxima vez que tengo que hacer la interrupción según el ID. Sumo 50 ms para dejar un colchón que permita que todos los robots oigan el mensaje antes de empezar a hablar.
        while(RTCisSyncing());                                                    //Espero la sincronización. 
        
        timeRecieved = true;                                                      //Levanto la bandera que indica que recibí el reloj del máster y ya puedo pasar a transmitir.
      }
  }

  //Loop de envío de la información del nodo.
  randomSeed(millis());                       //Antes de entrar al loop, genero la semilla de número aleatorios.
  while(timeRecieved){                        //Aquí solo debe enviar mensajes, pero eso lo hace la interrupción, así que aquí se construyen mensajes y se espera a que la interrupción los envíe.
    if(!mensajeCreado){
      float frac = 0.867;                     //Fracción que se usa para insertarle a los random decimales. Se escogió al azar, el número no significa nada.
      
      //Genero números al azar acorde a lo que el robot eventualmente podría enviar
      float robotID = (float)idRobot;         //Esta variable se debe volver a definir, pues idRobot al ser global presenta problema al crear el mensaje.
      float xP = (float)random(15,500)*frac;
      float yP = (float)random(1,500)*frac;
      float phi = 1.574;                      //Dato estático que se usó para ver la integridad del mensaje, el valor no significa nada.
      float tipo = (float)random(1,4);        //Este no lleva *frac porque el tipo en teoría es un 0,1,2 ó 3.
      float rObs = (float)random(1,50)*frac;
      float alphaObs = 281.649;               //Dato estático que se usó para ver la integridad del mensaje, el valor no significa nada.

      //Construyo mensaje (es una construcción bastante manual que podría mejorar)
      ptrMensaje = (uint32_t*)&robotID;       //Utilizo el puntero para extraer la información del dato flotante.
      for(uint8_t i = 0; i < 4; i++){
        mensaje[i] = (*ptrMensaje & (255UL << i*8)) >> i*8;  //La parte de "(255UL << i*8)) >> i*8" es solo para ir acomodando los bytes en el array de envío mensaje[].
      }

      ptrMensaje = (uint32_t*)&xP;
      for(uint8_t i = 4; i < 8; i++){
        mensaje[i] = (*ptrMensaje & (255UL << (i-4)*8)) >> (i-4)*8;
      }

      ptrMensaje = (uint32_t*)&yP;
      for(uint8_t i = 8; i < 12; i++){
        mensaje[i] = (*ptrMensaje & (255UL << (i-8)*8)) >> (i-8)*8;
      }

      ptrMensaje = (uint32_t*)&phi;
      for(uint8_t i = 12; i < 16; i++){
        mensaje[i] = (*ptrMensaje & (255UL << (i-12)*8)) >> (i-12)*8;
      }

      ptrMensaje = (uint32_t*)&tipo;
      for(uint8_t i = 16; i < 20; i++){
        mensaje[i] = (*ptrMensaje & (255UL << (i-16)*8)) >> (i-16)*8;
      }

      ptrMensaje = (uint32_t*)&rObs;
      for(uint8_t i = 20; i < 24; i++){
        mensaje[i] = (*ptrMensaje & (255UL << (i-20)*8)) >> (i-20)*8;
      }

      ptrMensaje = (uint32_t*)&alphaObs;
      for(uint8_t i = 24; i < 28; i++){
        mensaje[i] = (*ptrMensaje & (255UL << (i-24)*8)) >> (i-24)*8;
      }

      Serial.print(*(float*)&mensaje[0]); Serial.print("; "); Serial.print(*(float*)&mensaje[4]); Serial.print("; "); Serial.print(*(float*)&mensaje[8]); Serial.print("; "); Serial.print(*(float*)&mensaje[12]); Serial.print("; "); Serial.print(*(float*)&mensaje[16]); Serial.print("; "); Serial.print(*(float*)&mensaje[20]); Serial.print("; "); Serial.print(*(float*)&mensaje[24]); Serial.println(";"); 

      //Una vez creado el mensaje, no vuelvo a crear otro hasta que la interrupción baje la vandera.
      mensajeCreado = true;
    }
  }
  
}



/**** RTC FUNCIONES****/

inline bool RTCisSyncing(){
  //Función que lee el bit de sincronización de los registros
  return (RTC->MODE0.STATUS.bit.SYNCBUSY);
}

void config32kOSC(){
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
GCLK->GENDIV.reg = GCLK_GENDIV_ID(2)|GCLK_GENDIV_DIV(4);
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

void RTCdisable(){
  //Función que deshabilita el RTC.
  RTC->MODE0.CTRL.reg &= ~RTC_MODE0_CTRL_ENABLE; // disable RTC
  while (RTCisSyncing());
}

void RTCenable(){
  //Función que habilita el RTC.
  RTC->MODE0.CTRL.reg |= RTC_MODE0_CTRL_ENABLE; // enable RTC
  while (RTCisSyncing());
}

void RTCreset(){
  //Función que resetea el RTC.
  RTC->MODE0.CTRL.reg |= RTC_MODE0_CTRL_SWRST; // software reset
  while (RTCisSyncing());
}

void RTCresetRemove(){
  //Función que quita el reset del RTC.
  RTC->MODE0.CTRL.reg &= ~RTC_MODE0_CTRL_SWRST; // software reset remove
  while (RTCisSyncing());
}

void RTC_Handler(void){
  //Vector de interrupción.
  if(RTC->MODE0.COUNT.reg > 0x00000064){        //Si el valor del contador es mayor a 0x64 (100 en decimal) entonces haga la interrupción. Esto para evitar el problema que la interrupción se llame al puro inicio. No sé por qué pasaba esto, investigar más el tema.
    RTC->MODE0.COMP[0].reg += tiempoCicloTDMA;  //Quiero que haga una interrupción en el próximos ciclo del TDMA, actualizo el nuevo valor a comparar.  **¿Por qué debo agregar el [0]?**
    while(RTCisSyncing());                      //Llamo a función de escritura.
    
    rf69.send(mensaje, sizeof(mensaje));        //Llamo a la función de enviar de la biblioteca Radio Head para enviar el mensaje del nodo.
    mensajeCreado = false;                      //Bajo la bandera para indicar que ya se envió el mensaje y se puede crear uno nuevo.
    
    RTC->MODE0.INTFLAG.bit.CMP0 = true;         //Limpiar la bandera de la interrupción.
  }
}
