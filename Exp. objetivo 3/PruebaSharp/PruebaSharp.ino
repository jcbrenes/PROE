/** \file PROE_DETOB.ino
 * \author PROE
 * \link https://github.com/jcbrenes/PROE \endlink
 * \brief Código de detección de obstáculos
 * 
 *  
*/

#include <Servo.h>
#include <Wire_slave.h>   //para usar el STM32 como secundario en el puerto I2C                   
//https://github.com/rogerclarkmelbourne/Arduino_STM32/tree/master/STM32F1/libraries/WireSlave 

/************ Serial Setup ***************/
#define debug 1

#if debug == 1
#define serialPrint(x) Serial1.print(x)
#define serialPrintln(x) Serial1.println(x)
#else
#define serialPrint(x)
#define serialPrintln(x)
#endif

Servo myServo;

//Declaración de pines//
#define serv PB9 //Servo
#define sharp PA0 //Sharp (ir de distancia)

//Constantes de configuración//
const float beta=0.1;  //Constante para el filtro, ajustar para cambiar el comportamiento
const int servoDelay=5; //Tiempo entre cada paso del servo en milisegundos
const float distanciaMinimaSharp=140; //Distancia mínima en milimetros para detección de obstáculos
const int debounceTime=400; //Debounce para sensor IR frontal
const float maxTemp=50; //Temperatura de detección de fuego
const int movimientoMaximo=180; //Maxima rotacion en grados realizada por el servo que rota el sharp

//Variables globales//
int angulo=0;
int anguloAnterior=0;
int lectura=0;
float distancia=0;
float filtrado=0;
int cuentaBateria=0;
int sensorIRdetectado=0;
uint32_t millisAnterior=0;
int servoPos=0;
int incremento=1; //cantidad de grados que aumenta el servo en cada movimiento
bool ledON = false; 
int onTime= 300; //tiempo de encendido del LED en milisegundos
unsigned long millisLED=0;

int c=0; //Pulso de led para ver actividad del STM
unsigned long lastSend=0; //Almacena cuando se envio el ultimo obstaculo para evitar saturar la comunicación
const int sendDelay=100; //Tiempo minimo en ms entre cada envio de obstaculo (evita crash del stm)
char dato[50];
bool nuevoObstaculo = false;
unsigned long tiempoObstaculo=0; //Guarda en que momento se detecto el ultimo obstaculo
const int maxTiempoObstaculo=200; //Si no se atiende un obstaculo en 200ms se ignora

unsigned long lastPoll=0; //Almacena tiempo de ultima lectura de sensores IR
unsigned long pollingTime=100; //Tiempo minimo entre lectura de sonsores IR
int lastIR=0; //Ultimo sensor IR que se detecto, evita repetir mensajes

int contar = 0;

void setup() {
  myServo.attach(serv,500,1600); //Une el objeto myServo al pin del servo (serv, min, max) valores en us para mover el servo a su posición minima y máxima (calibración)

  //Configuración de pines//
  pinMode(sharp, INPUT);
  delay(1000); //Algunos sensores capturan ruido al inicio, los delays es para evitar eso

  //Configuración de perifericos//
  if (debug == 1){
    Serial1.begin(115200);  //Inicia la comunicación serial a 115200 baud
    while(!Serial1);
  }
  myServo.write(movimientoMaximo/2); //Coloca el servo en el centro para confirmar alineación
  //myServo.writeMicroseconds(1500);
  delay(1000); //Algunos sensores capturan ruido al inicio, los delays es para evitar eso
  serialPrintln("num;lectura;filtrado;distancia");
}
 
void loop() {
  if (contar < 500){
    revisarSharp(); //Serial1.println("Sharp");
    delay(10);
  }
}


void revisarSharp(){ //Revisa el valor del sensor Sharp  
  lectura=analogRead(sharp);
  filtrado=filtrado-(beta*(filtrado-lectura)); //Filtro de media movil
  distancia=1274160*pow(filtrado,-1.174); //Regresión lineal de datos experimentales para obtener distancia en milimetros
  serialPrint(contar);serialPrint(";");serialPrint(lectura);serialPrint(";");serialPrint(int(filtrado));serialPrint(";");serialPrintln(int(distancia));
  contar += 1;
}