//Código de calibración de MPU y magnetometro para ATTABOT PROE
//Código de calibracion del magnetometro de los robots del proyecto PROE basado en el paper: https://ieeexplore.ieee.org/abstract/document/4209540
//Usado en una placa Feather M0 RFM95 y magnetometro tipo _________
//https://github.com/jcbrenes/PROE

#include <Wire.h>
#include "wiring_private.h" // pinPeripheral() function

TwoWire myWire(&sercom1, 11, 13);

#define addr 0x0D //I2C Address for The HMC5883
#define PI 3.1415926535897932384626433832795 //Constante Pi
#define dirEEPROM B01010000 //Direccion de la memoria EEPROM

//Radios de conversión según data sheet del MPU6050
#define A_R 16384.0
#define G_R 131.0

float gx_off,gy_off,gz_off,acx_off,acy_off,acz_off;

//constantes del robot empleado
const int tiempoMuestreo = 10000; //unidades: micro segundos
const float pulsosPorRev = 206.0; //cantidad de pulsos de una única salida
const int factorEncoder = 4; //cantidad de tipos de pulsos que se están detectando (juego entre las 2 salidas del encoder)
const float circunferenciaRueda = 139.5; //Circunferencia de la rueda = 139.5mm
const float pulsosPorMilimetro = ((float)factorEncoder*pulsosPorRev) / circunferenciaRueda;
const float distanciaCentroARueda = 63.7; // Radio de giro del carro, es la distancia en mm entre el centro y una rueda.
const float conversionMicroSaMin = 1 / (60 * 1000000); // factor de conversion microsegundo (unidades del tiempo muestreo) a minuto
const float conversionMicroSaSDiv = 1000000; // factor de conversion microsegundo (unidades del tiempo muestreo) a segundo
const float tiempoMuestreoS = (float)tiempoMuestreo / conversionMicroSaSDiv;

//constantes para control PID de velocidad (están unidas con la constante de tiempo por simplificación de la ecuación)
const float velRequerida=180.0; //unidades mm/s
const float KpVel=0.90; //constante control proporcional
const float KiVel=1.0 * tiempoMuestreoS; //constante control integral
const float KdVel=0.01 / tiempoMuestreoS ; //constante control derivativo

//constantes para control PID de giro 
const float KpGiro=1.8; //constante control proporcional
const float KiGiro=20.0 * tiempoMuestreoS;//constante control integral
const float KdGiro=0.08 / tiempoMuestreoS; //constante control derivativo

//Constantes para la implementación del control PID real
const int errorMinIntegral = -250;
const int errorMaxIntegral = 250;
const int limiteSuperiorCicloTrabajoVelocidad = 200;
const int limiteInferiorCicloTrabajoVelocidad = 0;
const int limiteSuperiorCicloTrabajoGiro = 55;
const int limiteInferiorCicloTrabajoGiro = -55;
const int cicloTrabajoMinimo = 20;
const int minCiclosEstacionario = 20;

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

//variable que almacena el tiempo del último ciclo de muestreo
long tiempoActual = 0;

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

//Variables para el algoritmo de exploración
int unidadAvance = 100; //medida en mm que avanza cada robot por movimiento
int distanciaAvanzada = 0;
int poseActual[3] = {0, 0, 0}; //Almacena la pose actual: ubicación en x, ubicación en y, orientación.
bool giroTerminado = 1; //Se hace esta variable global para saber cuando se está en un giro y cuando no
bool obstaculoAdelante = false;
bool obstaculoDerecha = false;
bool obstaculoIzquierda = false;
enum orientacionesRobot {ADELANTE = 0, DERECHA = 90, IZQUIERDA = -90, ATRAS = 180}; //Se definen las orientaciones de los robots, el número indica la orientación de la pose
orientacionesRobot direccionGlobal = ADELANTE; //Se inicializa Adelante, pero en Setup se asignará un valor random
int anguloGiro = 0;

//Variables de calibración para magnetometro 
const int numSamples=700;
const double alfa=0.2;
const int desfase=(1-alfa)*20;
float rawx[numSamples]; //Lista de datos crudos en x
float rawy[numSamples]; //Lista de datos crudos en y
int j,i,k,l=0; //Contadores
float maxX,minX,maxY,minY; // Maximos y minimos de los datos
float xoff,yoff; //Guarda los offsets de los set de datos
int num1,num2,num3,num4; //Número de datos por cuadrante
float sumXX,sumYY,sumXY; //Sumas para los momentos de inercia
float uXX,uYY,uXY; //Momentos de inercia
float preAng,angulo; //Angulo de rotación de los datos
double factorEsc; //Factor de escalamiento para lograr un círculo
float xft,yft; //Valores filtrados

bool cal_mag=0;
bool cal_mpu=0;

void setup() {
  //asignación de pines
  pinMode(PWMA, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(13, OUTPUT); //LED del feather
  //asignación de interrupciones
  attachInterrupt(ENC_DER_C1, PulsosRuedaDerechaC1, CHANGE); //conectado el contador C1 rueda derecha
  attachInterrupt(ENC_DER_C2, PulsosRuedaDerechaC2, CHANGE);
  attachInterrupt(ENC_IZQ_C1, PulsosRuedaIzquierdaC1, CHANGE); //conectado el contador C1 rueda izquierda
  attachInterrupt(ENC_IZQ_C2, PulsosRuedaIzquierdaC2, CHANGE);
  //temporización y varibales aleatorias
  tiempoActual = micros(); //para temporización de los ciclos
  Serial.begin(115200);
  myWire.begin();
  // Assign pins 13 & 11 to SERCOM functionality
  pinPeripheral(11, PIO_SERCOM);
  pinPeripheral(13, PIO_SERCOM);
  inicializaMagnet();
  Serial.begin(115200);
  delay(3000);
}



void loop(){
  if (!cal_mpu){
    Calibracion_MPU();
  }

  if (!cal_mag){
    Calibracion_Mag(); 
  }
 
}

void Calibracion_Mag(){  //Función que calibra el magnetometro, realiza un giro de 360°, asegurarse que no tenga perturbaciones magneticas cerca en tiempo de calibración
  Serial.println("Calibrando Magnetometro");
  short x2,y2,z2;
  float x1,y1,d;
  medirMagnet(x2,y2,z2);
  //Toma las muestras con la función Giro() sobre una circunferencia completa
  while (!cal_mag){
    if(Giro(360)==false){
      if (i==0){ //La variable i definirá la cantidad de datos que se tomen
          rawx[i]=float(x2);
          rawy[i]=float(y2);
          i++;
        }
        //Algoritmo que funciona durante la toma de datos para eliminar datos redundantes para no exceder la RAM del feather
        else{
          x1=rawx[i-1];
          y1=rawy[i-1];
          d=sqrt(pow(abs(x2-x1),2)+pow(abs(y2-y1),2)); //Distancia euclideana entre 2 pares de puntos
          if (d>=1.0){
            rawx[i]=float(x2);
            rawy[i]=float(y2);
            i++;
          }
        }
      }
      //Etapa de filtrado, se utiliza el filtro de "Media movil"
     else{
       Serial.println("Procesando datos...");
       delay(300); 
       //Filtrado de datos
       for (int s=0;s<=i;s++){
        float crudox=rawx[s];
        float crudoy=rawy[s];
        xft=crudox*alfa+(1-alfa)*xft;
        yft=crudoy*alfa+(1-alfa)*yft;
        if (s>=desfase){ //El desfase se implementa para eliminar datos iniciales basura
          rawx[s-desfase]=xft;
          rawy[s-desfase]=yft; 
        }
      }
      //Calcula los maximos y mínimos
      maxX=maxF(rawx);
      minX=minF(rawx);
      maxY=maxF(rawy);
      minY=minF(rawy);
      //Calcula los offset del elipsoide y los sustrae
      xoff=(maxX+minX)/2;
      yoff=(maxY+minY)/2;
      for (int p=0;p<=i;p++){
          rawx[p]=rawx[p]-xoff;
          rawy[p]=rawy[p]-yoff;
      }
      //Determina los segundos momentos de inercia
      for (int h=0;h<=i;h++){
          sumXX=pow(rawx[h],2)+sumXX;
          sumYY=pow(rawy[h],2)+sumYY;
          sumXY=rawx[h]*rawy[h]+sumXY;
      }
      uXX=sumXX/i;
      uYY=sumYY/i;
      uXY=sumXY/i;
      //Calcula el angulo
      angulo=0.5*atan2((2*uXY),(uXX-uYY));
      //Escalado
      if ((maxX-minX)>(maxY-minY)){
        factorEsc=(maxX-minX)/(maxY-minY);
      }
      else{
        factorEsc=-1*(maxY-minY)/(maxX-minX);
      } 
     //Guarda valores en la memoria EEPROM
     guardarDatoFloat(xoff,0);
     guardarDatoFloat(yoff,4);
     guardarDatoFloat(angulo,8);
     guardarDatoFloat(factorEsc,12); 
     cal_mag=true;
     Serial.println("Magnetometro calibrado");
    } 
  }
}

void Calibracion_MPU(){  //Funcion que mide datos del MPU en posición horizontal y sin movimiento para calibracion y guarda la calibración en EEPROM
  Serial.println("Calibrando MPU");
  for(int j=0;j<=2;j++){
    float gx_prom=0;
    float gy_prom=0;
    float gz_prom=0;
    float acx_prom=0;
    float acy_prom=0;
    float acz_prom=0;
    for (int i=0;i<=100;i++){
      float gxx,gyy,gzz,axx,ayy,azz;
      leeMPU(gxx, gyy, gzz, axx, ayy, azz);
      acx_prom=acx_prom+axx;
      acy_prom=acy_prom+ayy;
      acz_prom=acz_prom+azz;
      gx_prom=gx_prom+gxx;
      gy_prom=gy_prom+gyy;
      gz_prom=gz_prom+gzz;
    }
   gx_off=gx_off+(gx_prom/100);
   gy_off=gy_off+(gy_prom/100);
   gz_off=gz_off+(gz_prom/100);
   acx_off=acx_off+(acx_prom/100);
   acy_off=acy_off+(acy_prom/100);
   acz_off=acz_off+(16384-(acz_prom/100));
  }
  //Guarda los datos en la EEPROM
  guardarDatoFloat(gx_off,16);
  guardarDatoFloat(gy_off,20);
  guardarDatoFloat(gz_off,24);
  guardarDatoFloat(acx_off,28);
  guardarDatoFloat(acy_off,32);
  guardarDatoFloat(acz_off,36);
  Serial.println("MPU calibrada");
  cal_mpu=true;
}






void inicializarMPU(){
  myWire.beginTransmission(0x68); //empezar comunicacion con el mpu6050
  myWire.write(0x6B);   //escribir en la direccion 0x6B
  myWire.write(0x00);   //escribe 0 en la direccion 0x6B (arranca el sensor)
  myWire.endTransmission();   //termina escritura
  }

//Funcione que extrae los datos crudos del MPU
void leeMPU(float &gx,float &gy,float &gz,float &ax,float &ay,float &az){
  
  int16_t gyro_x, gyro_y, gyro_z, tmp, ac_x, ac_y, ac_z; //datos crudos
  
  myWire.beginTransmission(0x68);   //empieza a comunicar con el mpu6050
  myWire.write(0x3B);   //envia byte 0x43 al sensor para indicar startregister
  myWire.endTransmission();   //termina comunicacion
  myWire.requestFrom(0x68,14); //pide 6 bytes al sensor, empezando del reg 43 (ahi estan los valores del giro)

  ac_x = myWire.read()<<8 | myWire.read();
  ac_y = myWire.read()<<8 | myWire.read();
  ac_z = myWire.read()<<8 | myWire.read();

  tmp = myWire.read()<<8 | myWire.read();

  gyro_x = myWire.read()<<8 | myWire.read(); //combina los valores del registro 44 y 43, desplaza lo del 43 al principio
  gyro_y = myWire.read()<<8 | myWire.read();
  gyro_z = myWire.read()<<8 | myWire.read();

  gx=float(gyro_x)-gx_off;
  gy=float(gyro_y)-gy_off;
  gz=float(gyro_z)-gz_off;

  ax=float(ac_x)-acx_off;
  ay=float(ac_y)-acy_off;
  az=float(ac_z)+acz_off;
}



bool Giro(float grados) {

  posActualRuedaDerecha = ConvDistAngular(contPulsosDerecha);
  int cicloTrabajoRuedaDerecha = ControlPosGiroRueda( grados, posActualRuedaDerecha, sumErrorGiroDer, errorAnteriorGiroDer );

  posActualRuedaIzquierda = ConvDistAngular(contPulsosIzquierda);
  int cicloTrabajoRuedaIzquierda = ControlPosGiroRueda( -grados, -posActualRuedaIzquierda, sumErrorGiroIzq, errorAnteriorGiroIzq);

  //Serial.print(cicloTrabajoRuedaDerecha);
  //Serial.print(",");
  //Serial.println(cicloTrabajoRuedaIzquierda);

  ConfiguraEscribePuenteH (cicloTrabajoRuedaDerecha, cicloTrabajoRuedaIzquierda);

  bool giroListo = EstadoEstacionario (cicloTrabajoRuedaDerecha, cicloTrabajoRuedaIzquierda, contCiclosEstacionarioGiro);

  return giroListo;

}

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

bool EstadoEstacionario (int pwmRuedaDer, int pwmRuedaIzq, int& contCiclosEstacionario) {
  //Función que revisa el estado de la acción de control y si se mantiene varios ciclos en cero, asume que ya está en el estado estacionario
  //Recibe los ciclos de trabajo en cada rueda y una variable por referencia donde se almacenan cuantos ciclos seguidos se llevan
  //Devuelve una variable que es TRUE si ya se alcanzó el estado estacionario

  bool estadoEstacionarioAlcanzado = false;

  if (pwmRuedaDer == 0 && pwmRuedaIzq == 0) {
    contCiclosEstacionario++;
    if (contCiclosEstacionario > minCiclosEstacionario) {
      ResetContadoresEncoders();
      contCiclosEstacionario = 0;
      estadoEstacionarioAlcanzado = true;
    }
  } else {
    contCiclosEstacionario = 0;
  }

  return estadoEstacionarioAlcanzado;
}

void ResetContadoresEncoders() {
  //Realiza una inicialización de las variables que cuentan cantidad de pulsos y desplazamiento de la rueda

  contPulsosDerecha = 0;
  contPulsosIzquierda = 0;

  posActualRuedaDerecha = 0.0;
  posActualRuedaIzquierda = 0.0;

  contPulsosDerPasado = 0;
  contPulsosIzqPasado = 0;

}

void PulsosRuedaDerechaC1() {
  //Manejo de interrupción del canal C1 del encoder de la rueda derecha
  LecturaEncoder(ENC_DER_C1, ENC_DER_C2, estadoEncoderDer, contPulsosDerecha);
}

void PulsosRuedaDerechaC2() {
  //Manejo de interrupción del canal C2 del encoder de la rueda derecha
  LecturaEncoder(ENC_DER_C1, ENC_DER_C2, estadoEncoderDer, contPulsosDerecha);
}

void PulsosRuedaIzquierdaC1() {
  //Manejo de interrupción del canal C1 del encoder de la rueda izquierda
  LecturaEncoder(ENC_IZQ_C1, ENC_IZQ_C2, estadoEncoderIzq, contPulsosIzquierda);
}

void PulsosRuedaIzquierdaC2() {
  //Manejo de interrupción del canal C2 del encoder de la rueda izquierda
  LecturaEncoder(ENC_IZQ_C1, ENC_IZQ_C2, estadoEncoderIzq, contPulsosIzquierda);
}

void LecturaEncoder(int entradaA, int entradaB, byte& state, int& contGiro) {
  //Función que determina si el motor está avanzando o retrocediendo en función del estado de las salidas del encoder y el estado anterior
  //Modifica la variable contadora de pulsos de ese motor

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


bool AvanzarDistancia(int distanciaDeseada) {
  //Avanza hacia adelante una distancia definida en mm a velocidad constante
  //Devuelve true cuando alcanzó la distancia deseada

  velActualDerecha = calculaVelocidadRueda(contPulsosDerecha, contPulsosDerPasado);
  int cicloTrabajoRuedaDerecha = ControlVelocidadRueda(velRequerida, velActualDerecha, sumErrorVelDer, errorAnteriorVelDer);

  velActualIzquierda = -1.0 * calculaVelocidadRueda(contPulsosIzquierda, contPulsosIzqPasado); //como las ruedas están en espejo, la vel es negativa cuando avanza, por eso se invierte
  int cicloTrabajoRuedaIzquierda = ControlVelocidadRueda(velRequerida, velActualIzquierda, sumErrorVelIzq, errorAnteriorVelIzq);

  Serial.print("Vel der: ");
  Serial.print(velActualDerecha, 5);
  Serial.print(" ; Vel izq: ");
  Serial.println(velActualIzquierda, 5);

  Serial.print("PWM der: ");
  Serial.print(cicloTrabajoRuedaDerecha);
  Serial.print(" ; PWM izq: ");
  Serial.println(cicloTrabajoRuedaIzquierda);

  ConfiguraEscribePuenteH (cicloTrabajoRuedaDerecha, cicloTrabajoRuedaIzquierda);

  float distanciaAvanzada = calculaDistanciaLinealRecorrida();

  bool avanceListo = false;
  if (distanciaAvanzada >= distanciaDeseada) {
    avanceListo = true;
    ResetContadoresEncoders();
  }

  return avanceListo;
}

void AvanzarIndefinido(int i, float &velIzq, float &velDer) {
  //Avanza hacia adelante indefinidamente
  //Luego de utilizarse, se debe llamar a ResetContadoresEncoders()

  velActualDerecha = calculaVelocidadRueda(contPulsosDerecha, contPulsosDerPasado);
  velDer = velActualDerecha;
  int cicloTrabajoRuedaDerecha = i;

  velActualIzquierda = -1.0 * calculaVelocidadRueda(contPulsosIzquierda, contPulsosIzqPasado); //como las ruedas están en espejo, la vel es negativa cuando avanza, por eso se invierte
  velIzq = velActualIzquierda;
  int cicloTrabajoRuedaIzquierda = i;
    
    Serial.print("Vel der: ");
    Serial.print(velActualDerecha,5);
    Serial.print(" ; Vel izq: ");
    Serial.println(velActualIzquierda,5);
   
  ConfiguraEscribePuenteH (cicloTrabajoRuedaDerecha, cicloTrabajoRuedaIzquierda);

}

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
  float pidTermVel = (KpVel * errorVel) + (KiVel * sumErrorVel) + (KdVel * difErrorVel);

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

void inicializaMagnet(){
  myWire.beginTransmission(addr); //start talking
  myWire.write(0x0B); // Tell the HMC5883 to Continuously Measure
  myWire.write(0x01); // Set the Register
  myWire.endTransmission();
  myWire.beginTransmission(addr); //start talking
  myWire.write(0x09); // Tell the HMC5883 to Continuously Measure
  myWire.write(0x1D); // Set the Register
  myWire.endTransmission();
  }

void medirMagnet(short &x,short &y,short &z){

  //Tell the HMC what regist to begin writing data into

  myWire.beginTransmission(addr);
  myWire.write(0x00); //start with register 3.
  myWire.endTransmission();

  //Read the data.. 2 bytes for each axis.. 6 total bytes
  myWire.requestFrom(addr, 6);
  if (6 <= myWire.available()) {
    x = myWire.read(); //LSB  x
    x |= myWire.read() << 8; //MSB  x
    y = myWire.read(); //LSB  y
    y |= myWire.read() << 8; //MSB y
    z = myWire.read(); //LSB z
    z |= myWire.read() << 8; //MSB z
  }
}

float maxF(float arrayData[]){
  float dato=0.0;
  float maxV=arrayData[0];
  for(int p=0;p<=i;p=p+1){
    dato=arrayData[p];
    if (dato>maxV){
      maxV=dato;
    }
  }
  return maxV; 
}

float minF(float arrayData[]){
  float dato=0;
  float minV=arrayData[0];
  for(int m=1;m<=i;m=m+1){
    dato=arrayData[m];
    if (dato<minV){
      minV=dato; 
    }
  }
  return minV; 
}




void guardarDatoFloat(float dato,int dirPagInicial){
    byte varr1; //primeros 8 bits para guardar en la EEPROM (LSB)
    byte varr2; //ultimo 8 bits para guardar en la EEPROM (MSB)
    byte varrDec; //parte decimal para guardar en la EEPROM
    byte signo; // signo del numero
    float dato2;
    if (dato<0.0){
      signo=1;
      dato2=(-1*dato);
    }
    else{
      signo=0;
      dato2=dato;
      } 
    transfVar(dato2,varr1,varr2,varrDec);
    eepromEscribe(dirEEPROM,dirPagInicial,varr1);
    delay(100);
    eepromEscribe(dirEEPROM,dirPagInicial+1,varr2);
    delay(100);
    eepromEscribe(dirEEPROM,dirPagInicial+2,varrDec);
    delay(100);
    eepromEscribe(dirEEPROM,dirPagInicial+3,signo);
    delay(100);
 }

float leerDatoFloat(int dirPagInicial){
    byte signo; //signo del numero, 1 para negativo, 0 para positivo
    byte varr1; //primeros 8 bits para guardar en la EEPROM (LSB)
    byte varr2; //ultimo 8 bits para guardar en la EEPROM (MSB)
    byte varrDec; //parte decimal para guardar en la EEPROM
    float dato;
    varr1=eepromLectura(dirEEPROM,dirPagInicial);
    varr2=eepromLectura(dirEEPROM,dirPagInicial+1);
    varrDec=eepromLectura(dirEEPROM,dirPagInicial+2);
    signo=eepromLectura(dirEEPROM,dirPagInicial+3);
    if (signo==1){
      dato=-1*constrVar(varr1,varr2,varrDec);
      return dato;
    }
    else{
      dato=constrVar(varr1,varr2,varrDec);
      return dato;
    }
}

float constrVar(byte LSB, byte MSB, byte dec){
    int piv1,piv2;
    float nuevo,decs;
    piv1=LSB;
    piv2=MSB<<8;
    nuevo=piv1|piv2;
    decs=float(dec);
    nuevo=nuevo+decs/100;
    return nuevo;
  }
  
void transfVar(float num,byte &var1,byte &var2,byte &varDec){
    int nuevoNum=int(num);
    int desplazamiento;
    float dec;
    desplazamiento=nuevoNum<<8;
    var1=desplazamiento>>8; //LSB
    var2=nuevoNum>>8; //MSB
    dec=100*(num-nuevoNum);
    varDec=int(dec);
  }

void eepromEscribe(byte dir, byte dirPag, byte data) {
  myWire.beginTransmission(dir);
  myWire.write(dirPag);
  myWire.write(data);
  myWire.endTransmission();
}

byte eepromLectura(int dir, int dirPag) {
  myWire.beginTransmission(dir);
  myWire.write(dirPag);
  myWire.endTransmission();

  myWire.requestFrom(dir, 1);
  if(myWire.available())
    return myWire.read();
  else
    return 0xFF;
}
