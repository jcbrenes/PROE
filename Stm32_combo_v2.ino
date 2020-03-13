/*
Código para STM32 de PROE v1
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
#define led2 PB4 //Led adicional 2
#define led3 PB5 //Led adicional 3
#define serv PB9 //Servo
#define temp PB1 //Sensor de temperatura
#define int1 PA4 //Pin de interrupción para la feather
#define sharp PA0 //Sharp (ir de distancia)


//Constantes de configuración//
const float beta=0.1;  //Constante para el filtro, ajustar para cambiar el comportamiento
const int servoDelay=15; //Tiempo entre cada paso del servo en milisegundos
const float distanciaMinimaSharp=5; //Distancia mínima en milimetros para detección de obstáculos
const int debounceTime=400; //Debounce para sensor IR frontal
const float maxTemp=40; //Temperatura de detección de fuego
const int movimientoMaximo=120; //Maxima rotacion en grados realizada por el servo que rota el sharp

//Variables globales//
int angulo=0;
int lectura=0;
float distancia=0;
float filtrado=0;
int cuentaBateria=0;



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

  //Configuración de interrupciones//
  attachInterrupt(irFrontal, irFrontalSend, FALLING); //Activo bajo
  attachInterrupt(irLeft, irLeftSend, RISING); //Activo alto
  attachInterrupt(irRight, irRightSend, RISING); //Activo alto

  //Configuración de perifericos//
  Wire.begin();           //Inicia comunicación i2c como master
  Serial1.begin(115200);  //Inicia la comunicación serial a 115200 baud
  Serial1.println("On");  //Enviar un "on" por el serial para confirmar que el setup se ejecutó correctamente
}
 
void loop() {
  moverServo();
  revisarTemperatura();
  revisarBateria();
}

void moverServo(){ //Mueve el servo un grado y mide la distancia
  for(int i=0;i<movimientoMaximo;i+=1){
    myServo.write(i);
    angulo=i;
    delay(servoDelay);
    lectura=analogRead(sharp);
    filtrado=filtrado-(beta*(filtrado-lectura)); //Filtro de media movil
    distancia=1274160*pow(filtrado,-1.174); //Regresión lineal de datos experimentales para obtener distancia en milimetros
    //Serial1.println(distancia);
    if (distancia<distanciaMinimaSharp){
      enviarDato(1,int(distancia),angulo); //Enviar obstaculo tipo 1 con distancia en cm y angulo
    }
  }
  for(int i=movimientoMaximo;i>0;i-=1){
    myServo.write(i);
    angulo=i;
    delay(servoDelay);
    lectura=analogRead(sharp);
    filtrado=filtrado-(beta*(filtrado-lectura));
    distancia=1274160*pow(filtrado,-1.174);
    //Serial1.println(distancia);
    if (distancia<distanciaMinimaSharp){
      enviarDato(1,int(distancia),angulo); //Enviar obstaculo tipo 1 con distancia en cm y angulo
    }
  }
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
  if(digitalRead(lowBattery)){
    cuentaBateria++;
    if(cuentaBateria>=20){ //Si la bateria esta baja enviar advertencia cada 20 ciclos del loop principal
      enviarDato(6,0,0);
      cuentaBateria=0;
    }
  }
}

void enviarDato(int caso, int distancia, int angulo){ //Envia el paquete de datos con la información de obstaculo detectado
  digitalWrite(int1,HIGH); //Pulsar salida int1 para indicarle al feather que se detectó un obstaculo y luego enviar los detalles
  delay(2); //Delay para darle tiempo al feather de que llegue a la interrupcion y se ponga a leer el i2c, puede que no sea necesario
  digitalWrite(int1,LOW);
  char dato[50];
  sprintf(dato, "%d,%d,%d.", caso, distancia, angulo); //Genera un string para la transmisión
  Wire.beginTransmission(42);           //Comienza transmisión al esclavo 42 (Feather)
  Wire.write(dato);                     //Envia el valor al esclavo
  Wire.endTransmission();               //Detener la transmisión
}

//Funciones de interrupción//

void irFrontalSend(){ //Interrupcion para sensor frontal, codigo 2
  //El sensor frontal es extremadamente ruidoso por lo que requiere un filtrado adicional
  static unsigned long last_interrupt_time = 0;
  unsigned long interrupt_time = millis();
  
  if (interrupt_time - last_interrupt_time > debounceTime) //Debounce para el sensor frontal
  {
    if(!digitalRead(irFrontal)){
      enviarDato(2,0,0);
      Serial1.println("F");
    }
      
  }
  last_interrupt_time = interrupt_time;
}

void irRightSend(){ //Interrupcion para sensor derecho, codigo 3
  enviarDato(3,0,0);
  Serial1.println("R");
}

void irLeftSend(){ //Interrupcion para sensor izquierdo, codigo 4
  enviarDato(4,0,0);
  Serial1.println("L");
}
