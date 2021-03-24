//Código de feather base que recibe datos de los robots en el campo y los envia por usb al puerto serial en el formato predefinido


#include <SPI.h>
#include <RH_RF69.h>
#include <RHReliableDatagram.h>

/************ Radio Setup ***************/
#define RF69_FREQ 915.0   // Frecuencias deben ser iguales con respecto a los demás nodos
#define MY_ADDRESS     1  // Dirección del receptor (base). No sé si es necesaria.
// Definiciones de pines.
#define RFM69_CS      8
#define RFM69_INT     3
#define RFM69_RST     4
#define LED           13

// Inicialización del driver de la biblioteca Radio Head.
RH_RF69 rf69(RFM69_CS, RFM69_INT);
RHReliableDatagram rf69_manager(rf69, MY_ADDRESS);

unsigned long timeStamp1;
uint8_t buf[RH_RF69_MAX_MESSAGE_LEN];

int c=0;

void setup() 
{
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
  
  if (!rf69_manager.init()) {
    Serial.println("RFM69 inicialización fallida");
    while (1);
  }
  Serial.println("RFM69 radio init OK!");
  
  // Setear frecuencia
  if (!rf69.setFrequency(RF69_FREQ)) {
    Serial.println("setFrequency failed");
  }

  // Configurar potencia
  rf69.setTxPower(20, true);   //Rango de 14-20 para la potencia, segundo argumento debe ser verdadero para el 69HCW.
  // Configurar aL NODO para que escuche cualquier dirección (esto sí es necesario).

  

  // The encryption key has to be the same as the one in the server
  uint8_t key[] = { 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
                    0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};
  rf69.setEncryptionKey(key);

  Serial.print("RFM69 radio @");  Serial.print((int)RF69_FREQ);  Serial.println(" MHz");
  
  unsigned long timeStamp1 = RTC->MODE0.COUNT.reg; //Extraer tiempo del RTC de la base
  uint8_t reloj[3];
  uint32_t* ptrReloj;  
  ptrReloj = (uint32_t*)&timeStamp1;       //Utilizo el puntero para extraer la información del dato flotante.
  
  for(uint8_t i = 0; i < 4; i++){
    reloj[i] = *ptrReloj >> i*8;  //La parte de "(255UL << i*8)) >> i*8" es solo para ir acomodando los bytes en el array de envío mensaje[].
  }
  rf69_manager.sendtoWait(reloj, sizeof(reloj), 4);     //Enviar valor del RTC al esclavo
  //Esta función envia un solo mensaje y asume que todos los esclavos la reciben al mismo tiempo en la misma dirección
  //Podria ocasionar problemas si uno de los robot no recibe la señal requeriria un reinicio de todo el sistema
}


void loop() {
  uint8_t len = sizeof(buf);

  if (rf69.recv(buf, &len, timeStamp1)) {
    if ((*(float*)&buf[0])<1) return; //Si el ID es 0 no lo muestra, evita enviar un array vacio a consola
    buf[len] = 0;
    Serial.print(*(float*)&buf[0]); Serial.print("; "); Serial.print(*(float*)&buf[4]); Serial.print("; "); Serial.print(*(float*)&buf[8]); Serial.print("; "); Serial.print(*(float*)&buf[12]); Serial.print("; "); Serial.print(*(float*)&buf[16]); Serial.print("; "); Serial.print(*(float*)&buf[20]); Serial.print("; "); Serial.print(*(float*)&buf[24]); Serial.println(";");
  }

  if(Serial.available()){ //Si se recibe algo por el serial vuelve a mandar la señal de sincronización, evita tener que reiniciar el feather
    while(Serial.available()){ //Limpiar el buffer del serial
      Serial.read();
    }
    sincronizar();
  }
  
  actividad();
}

void sincronizar(){
  unsigned long timeStamp1 = RTC->MODE0.COUNT.reg; //Extraer tiempo del RTC de la base
  uint8_t reloj[3];
  uint32_t* ptrReloj;  
  ptrReloj = (uint32_t*)&timeStamp1;       //Utilizo el puntero para extraer la información del dato flotante.
  
  for(uint8_t i = 0; i < 4; i++){
    reloj[i] = *ptrReloj >> i*8;  //La parte de "(255UL << i*8)) >> i*8" es solo para ir acomodando los bytes en el array de envío mensaje[].
  }
  rf69_manager.sendtoWait(reloj, sizeof(reloj), 4);     //Enviar valor del RTC al esclavo
}

void actividad(){  //Pulsar led 13 para mostrar actividad del feather
  if(c>100000){
    digitalWrite(13,!digitalRead(13));
    c=0;
  }
  else{
    c=c+1;
  }
}
