//Código de detección de obstáculos para el proyecto PROE
//Usado en la placa Bluepill STM32
//https://github.com/jcbrenes/PROE
/*
Se encarga de la lectura de sensores y comunica cualquier evento mediante i2c al feather al que está conectado despues de notificarle mediante un pin adicional de interurpcion
Se usa el formato "x,y,z." donde "x" es el código del evento, "y" es el valor de lectura si aplica y "z" el angulo en caso del sensor Sharp.
Códigos de evento:
  1)Sensor Sharp
  2)IR frontal
  3)IR derecho
  4)IR izquierdo
  5)Temperatura
  6)Batería baja
Adicionalmente tiene tres salidas conectadas a leds para notificación y un switch extra
*/

#include <Servo.h>
#include <Wire.h>                      
#include <SoftWire.h>  //Biblioteca para comunicación i2c

Servo myServo;

//Declaración de pines//
#define irLeft PB12 //Sensor ir izquierda
#define irFrontal PB13 //Sensor ir frontal
#define irRight PB14 //Sensor ir derecha
#define lowBattery PB15 //Pin de batería baja del boost
#define interruptor PA8 //Interruptor adicional, usar pullup de software
#define led1 PB3 //Led adicional 1
#define led2 PA3 //Led adicional 2
#define led3 PA5 //Led adicional 3
#define serv PB9 //Servo
#define temp PB1 //Sensor de temperatura
#define int1 PA4 //Pin de interrupción para la feather
#define sharp PA0 //Sharp (ir de distancia)


//Constantes de configuración//
const float beta=0.1;  //Constante para el filtro, ajustar para cambiar el comportamiento
const int servoDelay=5; //Tiempo entre cada paso del servo en milisegundos
const float distanciaMinimaSharp=130; //Distancia mínima en milimetros para detección de obstáculos
const int debounceTime=400; //Debounce para sensor IR frontal
const float maxTemp=50; //Temperatura de detección de fuego
const int movimientoMaximo=110; //Maxima rotacion en grados realizada por el servo que rota el sharp

//Variables globales//
int angulo=0;
int anguloAnterior=0;
int lectura=0;
float distancia=0;
float filtrado=0;
int cuentaBateria=0;
int sensorIRdetectado=0;
unsigned long millisAnterior=0;
int servoPos=0;
int incremento=1; //cantidad de grados que aumenta el servo en cada movimiento
bool ledON = false; 
int onTime= 300; //tiempo de encendido del LED en milisegundos
unsigned long millisLED=0;

int c=0; //Pulso de led para ver actividad del STM


void setup() {
  myServo.attach(serv); //Une el objeto myServo al pin del servo (serv)

  //Configuración de pines//
  pinMode(irFrontal, INPUT);
  pinMode(irLeft, INPUT);
  pinMode(irRight, INPUT);
  pinMode(interruptor, INPUT_PULLUP);
  pinMode(lowBattery, INPUT);
  pinMode(led1, OUTPUT);
  pinMode(led2, OUTPUT);
  pinMode(led3, OUTPUT);
  pinMode(temp, INPUT);
  pinMode(int1, OUTPUT);
  pinMode(sharp, INPUT);

  delay(1000); //Algunos sensores capturan ruido al inicio, los delays es para evitar eso
  
  //Configuración de interrupciones//
  attachInterrupt(irFrontal, irFrontalSend, FALLING); //Activo bajo
  attachInterrupt(irLeft, irLeftSend, RISING); //Activo alto
  attachInterrupt(irRight, irRightSend, RISING); //Activo alto

  //Configuración de perifericos//
  Wire.begin();           //Inicia comunicación i2c como master
  Serial1.begin(115200);  //Inicia la comunicación serial a 115200 baud
  Serial1.println("On");  //Enviar un "on" por el serial para confirmar que el setup se ejecutó correctamente

  myServo.write(movimientoMaximo/2); //Coloca el servo en el centro para confirmar alineación
  delay(1000); //Algunos sensores capturan ruido al inicio, los delays es para evitar eso
}
 
void loop() {
  moverServo();
  revisarSharp();
  revisarSensoresIR();
  //revisarTemperatura(); //desactivado mientras no está soldado el sensor al PCB
  revisarBateria();
  encenderLED(); //enciende un led si alguna función lo activó 
  actividad(); //Parpadea led amarillo para confirmar actividad del stm
  
}

void moverServo(){ //Mueve el servo un valor determinado cada cierto tiempo
  if((millis() - millisAnterior) > servoDelay) {  //Se utiliza la función millis para evitar usar los delays
    millisAnterior = millis();
    servoPos += incremento;
    myServo.write(servoPos);
    angulo= movimientoMaximo/2 - servoPos; //se considera 0 grados justo al frente
    if ((servoPos >= movimientoMaximo) || (servoPos <= 0)){ // se terminó el semiciclo, se invierte la dirección
      incremento = -incremento;
    }
  }
}

void encenderLED(){ //Se encarga de encender el LED por un tiempo determinado. Se utiliza la función millis para evitar usar los delays
  if(ledON){ //enciende el LED
    digitalWrite(led3,HIGH); 
    millisLED = millis();
    ledON = false; 
  }
  if((millis() - millisLED) > onTime) {  //Se apaga el LED luego de un rato
    digitalWrite(led3,LOW); 
  }
}

void revisarSharp(){ //Revisa el valor del sensor Sharp
    lectura=analogRead(sharp);
    filtrado=filtrado-(beta*(filtrado-lectura)); //Filtro de media movil
    distancia=1274160*pow(filtrado,-1.174); //Regresión lineal de datos experimentales para obtener distancia en milimetros
    if ((distancia<distanciaMinimaSharp) & (angulo!=anguloAnterior)){ //revisa distancia umbral y que solo se haga un envío por cada ángulo
      enviarDato(1,int(distancia),angulo); //Enviar obstaculo tipo 1 con distancia en cm y angulo
    }
    anguloAnterior=angulo;
}

void revisarTemperatura(){ //Lectura del sensor de temperatura
  float lectura = analogRead(temp);
  float temperatura = (0.08394*lectura)-54.162; //Formula extraida de la hoja de datos del sensor
  //Serial1.println(lectura);
  //Serial1.println(temperatura);
  if(temperatura>maxTemp){ //Valor máximo de temperatura aceptable
    int i = int(temperatura);
    enviarDato(5,i,0);
    Serial1.println("T");
  }
}

void revisarBateria(){ //Revisar la batería cada 20 ciclos del loop, codigo 6
  if(!digitalRead(lowBattery)){ //Batería baja = pin bajo
    cuentaBateria++;
    if(cuentaBateria>=10000){ //Si la bateria esta baja enviar advertencia cada 10000 ciclos del loop principal
      //enviarDato(6,0,0);
      ledON=true;
      onTime=300;
      cuentaBateria=0;
    }
  }
}

void revisarSensoresIR(){ //Revisa si la variable de sensores IR cambio de estado en un interupción, en ese caso envía el dato
//Esto es porque los serial y wire dentro de interrupciones dan problema

  switch (sensorIRdetectado) {
    case 0:    // no se ha detectado nada
      break;
    case 2:    // sensor IR frontal
      enviarDato(2,0,0);
      sensorIRdetectado=0;
      Serial1.print("F ");
      break;
    case 3:    // sensor IR inferior derecho
      enviarDato(3,0,0);
      sensorIRdetectado=0;
      Serial1.print("R ");
      break;
    case 4:    // sensor IR inferior izquierdo
      enviarDato(4,0,0);
      sensorIRdetectado=0;
      Serial1.print("L ");
      break;
  }
}

void enviarDato(int caso, int distancia, int angulo){ //Envia el paquete de datos con la información de obstaculo detectado

  //Solo si el obstáculo está en frente activa el pin de interrupción para el feather
  if (abs(angulo)<=45){
    digitalWrite(int1,HIGH); //Pulsar salida int1 para indicarle al feather que se detectó un obstaculo y luego enviar los detalles
    delay(2); //Delay para darle tiempo al feather de que llegue a la interrupcion y se ponga a leer el i2c, puede que no sea necesario
    digitalWrite(int1,LOW);  
  }

  char dato[50];
  sprintf(dato, "%d,%d,%d.", caso, distancia, angulo); //Genera un string para la transmisión
  Wire.beginTransmission(42);           //Comienza transmisión al esclavo 42 (Feather)
  Wire.write(dato);                     //Envia el valor al esclavo
  Wire.endTransmission();               //Detener la transmisión
  //para visualización, enciende led y muestra el dato
  ledON=true;
  onTime=1000;
  Serial1.println(dato);
  
}

void actividad(){
  if(c>5000){//Parpadea led amarillo para señalar actividad del stm
    digitalWrite(led2,!digitalRead(led2));
    c=0;
  }
  else{
    c=c+1;
  }
}

//Funciones de interrupción//

void irFrontalSend(){ //Interrupcion para sensor frontal, codigo 2
  //El sensor frontal es extremadamente ruidoso por lo que requiere un filtrado adicional
  static unsigned long last_interrupt_time = 0;
  unsigned long interrupt_time = millis();
  
  if (interrupt_time - last_interrupt_time > debounceTime) //Debounce para el sensor frontal
  {
    if(!digitalRead(irFrontal)){
      sensorIRdetectado=2;
    }
  }
  last_interrupt_time = interrupt_time;
}

void irRightSend(){ //Interrupcion para sensor derecho, codigo 3
  sensorIRdetectado=3;
}

void irLeftSend(){ //Interrupcion para sensor izquierdo, codigo 4
  sensorIRdetectado=4;
}
