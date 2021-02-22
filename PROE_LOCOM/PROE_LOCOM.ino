//Código de locomoción y comunicacion para el proyecto PROE
//Usado en la placa Feather M0 RFM95
//https://github.com/jcbrenes/PROE

#include<Wire.h> //Bilbioteca para la comunicacion I2C
#include "wiring_private.h" // pinPeripheral() function
 
TwoWire myWire(&sercom1, 11, 13);

#define dirEEPROM B01010000 //Direccion de la memoria EEPROM
#define addr 0x0D //I2C Address para el HMC5883

//constantes del robot empleado
const int tiempoMuestreo=10000; //unidades: micro segundos
const float pulsosPorRev=206.0; //cantidad de pulsos de una única salida
const int factorEncoder=4; //cantidad de tipos de pulsos que se están detectando (juego entre las 2 salidas del encoder)
const float circunferenciaRueda=139.5;//Circunferencia de la rueda = 139.5mm 
const float pulsosPorMilimetro=((float)factorEncoder*pulsosPorRev)/circunferenciaRueda; 
const float distanciaCentroARueda=87.5;// Radio de giro del carro, es la distancia en mm entre el centro y una rueda. 
const float conversionMicroSaMin=1/(60 * 1000000);// factor de conversion microsegundo (unidades del tiempo muestreo) a minuto
const float conversionMicroSaSDiv=1000000;// factor de conversion microsegundo (unidades del tiempo muestreo) a segundo
const float tiempoMuestreoS= (float)tiempoMuestreo/conversionMicroSaSDiv;

//constantes para control PID de velocidad (están unidas con la constante de tiempo por simplificación de la ecuación)
const float velRequerida=150; //unidades mm/s
const float KpVel=2; //constante control proporcional
const float KiVel=20.0 * tiempoMuestreoS; //constante control integral
const float KdVel=0.01 / tiempoMuestreoS ; //constante control derivativo
//constantes para control PID de giro 
const float KpGiro=1.8; //constante control proporcional
const float KiGiro=20.0 * tiempoMuestreoS;//constante control integral
const float KdGiro=0.08 / tiempoMuestreoS; //constante control derivativo

//Constantes para la implementación del control PID real
const int errorMinIntegral=-250;
const int errorMaxIntegral=250;
const int limiteSuperiorCicloTrabajoVelocidad=200;
const int limiteInferiorCicloTrabajoVelocidad=0;
const int limiteSuperiorCicloTrabajoGiro=200;
const int limiteInferiorCicloTrabajoGiro=-200;
const int cicloTrabajoMinimo= 20;
const int minCiclosEstacionario= 20;

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

//Configuracion de los pines de interrupcion de Obstáculos
const int INT_OBSTACULO = A4;

//Almacenamiento de datos de obstáculos
const int longitudArregloObstaculos = 1000;

//Constantes algoritmo exploración
int unidadAvance= 200; //medida en mm que avanza cada robot por movimiento

//VARIABLES GLOBALES

//Variables para la máquina de estados principal
enum PosiblesEstados {AVANCE=0, GIRE_DERECHA, GIRE_IZQUIERDA,ESCOGER_DIRECCION,GIRO, NADA};
char *PosEstados[] = {"AVANCE", "GIRE_DERECHA", "GIRE_IZQUIERDA","ESCOGER_DIRECCION","GIRO", "NADA"};
PosiblesEstados estado = AVANCE;

//variable que almacena el tiempo del último ciclo de muestreo
long tiempoActual=0;

//contadores de pulsos del encoder
int contPulsosDerecha=0;
int contPulsosIzquierda=0;
int contPulsosDerPasado=0;
int contPulsosIzqPasado=0;

//Variables que almacenan el desplazamiento angular de cada rueda
float posActualRuedaDerecha=0.0;
float posActualRuedaIzquierda=0.0;

//Variables del valor de velocidad en cada rueda
float velActualDerecha=0.0;
float velActualIzquierda=0.0;

//Valores acumulados para uso en las ecuaciones de control
//Para Giro
float errorAnteriorGiroDer=0;
float errorAnteriorGiroIzq=0;
float sumErrorGiroDer=0;
float sumErrorGiroIzq=0;
int contCiclosEstacionarioGiro=0;
//Para Velocidad
float errorAnteriorVelDer=0;
float errorAnteriorVelIzq=0;
float sumErrorVelDer=0;
float sumErrorVelIzq=0;

//Variables para las interrupciones de los encoders
byte estadoEncoderDer=1; 
byte estadoEncoderIzq=1;

//Variables para el almacenar datos de obstáculos
int datosSensores[longitudArregloObstaculos][6]; //Arreglo que almacena la información de obstáculos de los sensores (tipo sensor, distancia, ángulo)
int ultimoObstaculo=-1; //Apuntador al último obstáculo en el arreglo. Se inicializa en -1 porque la función de guardar aumenta en 1 el índice

//Variables para el algoritmo de exploración
int distanciaAvanzada=0;
int poseActual[3]={0,0,0}; //Almacena la pose actual: ubicación en x, ubicación en y, orientación.
bool giroTerminado=1; //Se hace esta variable global para saber cuando se está en un giro y cuando no
bool obstaculoAdelante=false;
bool obstaculoDerecha=false;
bool obstaculoIzquierda=false;
enum orientacionesRobot {ADELANTE=0, DERECHA=90, IZQUIERDA=-90, ATRAS=180}; //Se definen las orientaciones de los robots, el número indica la orientación de la pose
orientacionesRobot direccionGlobal = ADELANTE; //Se inicializa Adelante, pero en Setup se asignará un valor random
int anguloGiro = 0;

//Variables para la calibración
float alfaIzq=1;
float betaIzq=0;
float alfaDer=1;
float betaDer=0;

void setup() {
  //asignación de pines
  pinMode(PWMA, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(INT_OBSTACULO, INPUT_PULLUP);
  pinMode(13,OUTPUT); //LED del feather
  delay(1000); //delay para evitar interrupciones al arrancar
  //asignación de interrupciones
  attachInterrupt(ENC_DER_C1, PulsosRuedaDerechaC1,CHANGE);  //conectado el contador C1 rueda derecha
  attachInterrupt(ENC_DER_C2, PulsosRuedaDerechaC2,CHANGE); 
  attachInterrupt(ENC_IZQ_C1, PulsosRuedaIzquierdaC1,CHANGE);  //conectado el contador C1 rueda izquierda
  attachInterrupt(ENC_IZQ_C2, PulsosRuedaIzquierdaC2,CHANGE);
  attachInterrupt(digitalPinToInterrupt(INT_OBSTACULO), DeteccionObstaculo,FALLING);
  //temporización y varibales aleatorias
  tiempoActual=micros(); //para temporización de los ciclos
  randomSeed(analogRead(A5)); //Para el algoritmo de exploración, el pinA5 está al aire
  direccionGlobal= (orientacionesRobot)(random(-1,2)*90); //Se asigna aleatoriamente una dirección global a seguir por el algoritmo RWD
  //Inicialización de puertos seriales
  Serial.begin(115200);
  Wire.begin(42); // En el puerto I2c se asigna esta dirección como esclavo
  Wire.onReceive(RecibirI2C);
  pinPeripheral(11, PIO_SERCOM);
  pinPeripheral(13, PIO_SERCOM);
  delay(2000);
}

void loop(){
  //Las acciones de la máquina de estados y los controles se efectuarán en tiempos fijos de muestreo
  if((micros()-tiempoActual)>=tiempoMuestreo){
     tiempoActual=micros();
    
     //Máquina de estados que cambia el modo de operación
     switch (estado) {
  
        case AVANCE:  { 
          bool avanceTerminado= AvanzarDistancia(unidadAvance); 
          if (avanceTerminado){
            estado = ESCOGER_DIRECCION; 
          }        
          break; 
        }
        
        case GIRE_DERECHA: { 
          giroTerminado= Giro(90);
          if   (giroTerminado) {
            ConfiguracionParar(); //detiene el carro un momento
            estado = GIRE_IZQUIERDA;
          }
          break; 
        }
        
        case GIRE_IZQUIERDA:  { 
          giroTerminado= Giro(-90);
          if   (giroTerminado) {
            ConfiguracionParar(); //detiene el carro un momento
            estado = GIRE_DERECHA;
          }        
          break; 
        }
        
        case ESCOGER_DIRECCION: {
          ActualizarUbicacion(); //Primero actualiza la ubicación actual en base al avance anterior y la orientación actual
          ConfiguracionParar(); //detiene el carro un momento
          RevisaObstaculoPeriferia(); //Revisa los osbtáculos presentes en la pose actual
          AsignarDireccionRWD(); //Asigna un ángulo de giro en base al algoritmo Random Walk con Dirección
          estado= GIRO;
        }
        
        case GIRO: {
          giroTerminado=Giro((float)anguloGiro);
          if(giroTerminado){
            digitalWrite(13,LOW);
            poseActual[2]= poseActual[2] + anguloGiro; //Actualiza la orientación. Supongo que no se va a detener un giro a la mitad por un obstáculo
            ConfiguracionParar();
            estado=AVANCE;
          }
        }
        
        case NADA: { 
          break; 
        }
     }
  }
}

void RecibirI2C (int cantidad)  { 
//Función (tipo Evento) llamada cuando se recibe algo en el puerto I2C conectado al STM32
//Almacena en una matriz las variables de tipo de sensor, distancia y ángulo
  int cont=1;
  int tipoSensor=0;
  int distancia=0;
  int angulo=0;
  String acumulado = "";
  
  while(0 < Wire.available()) { // ciclo mientras se reciben todos los datos
    char c = Wire.read(); // se recibe un byte a la vez y se maneja como char
    if (c==','){ //los datos vienen separados por coma 
      if (cont==1) {
        tipoSensor = acumulado.toInt();
        acumulado = "";
      } 
      if (cont==2) {
        distancia = acumulado.toInt();
        acumulado = "";
      }
      cont++;
    }else if (c=='.') { //el ultimo dato viene con punto al final
      angulo = acumulado.toInt();
      acumulado = "";
      cont=1;
    } else {
      acumulado += c;  //añade el caracter a la cadena anterior
    }
  }
  
  //Almacenamiento de datos en el arreglo de datos de los sensores
  if (ultimoObstaculo == longitudArregloObstaculos-1){ //si llega al final del arreglo regresa al inicio, sino suma 1
    ultimoObstaculo=0;
  }else {
    ultimoObstaculo++; 
  }
  datosSensores[ultimoObstaculo][0]= poseActual[0]; //Guarda la pose actual donde se detectó el obstáculo
  datosSensores[ultimoObstaculo][1]= poseActual[1];
  datosSensores[ultimoObstaculo][2]= poseActual[2];
  datosSensores[ultimoObstaculo][3]= tipoSensor;
  datosSensores[ultimoObstaculo][4]= distancia;
  datosSensores[ultimoObstaculo][5]= angulo;
}

void DeteccionObstaculo(){
//Función tipo interrupción llamada cuando se activa el pin de detección de obstáculo del STM32
//Son obstáculos que requieren que el robot cambie de dirección

  if(giroTerminado==1){ //Solo se atiende interrupción si no está haciendo un giro, sino todo sigue igual
   digitalWrite(13,HIGH);
   Serial.print("INT OBS!  ");
   Serial.print(datosSensores[ultimoObstaculo][3]);
   Serial.print("  d: ");
   Serial.print(datosSensores[ultimoObstaculo][4]);
   Serial.print("  ang: ");
   Serial.println(datosSensores[ultimoObstaculo][5]);
   delay(10);
   estado=ESCOGER_DIRECCION;
  }   
}

void ActualizarUbicacion(){
//Función que actualiza la ubicación actual en base al avance anterior y la orientación actual
  distanciaAvanzada= (int)calculaDistanciaLinealRecorrida();
  if (poseActual[2]==0 || abs(poseActual[2]==360)){ poseActual[1]= poseActual[1] + distanciaAvanzada;}
  else if (poseActual[2]==90 || poseActual[2]==-270) { poseActual[0] = poseActual[0] + distanciaAvanzada;}
  else if (poseActual[2]==-90 || poseActual[2]==270) { poseActual[0] = poseActual[0] - distanciaAvanzada;}
  else if (abs(poseActual[2])==180) { poseActual[1] = poseActual[1] - distanciaAvanzada;}
  Serial.print("Pose=>  X: ");
  Serial.print(poseActual[0]);
  Serial.print("  Y: ");
  Serial.print(poseActual[1]);
  Serial.print("  Theta: ");
  Serial.println(poseActual[2]);
  delay(10);
}

void RevisaObstaculoPeriferia(){
//Función que revisa los obstáculos detectados previamente y determina cuales están en la periferia actual
//Retorna una simplificación de si hay ostáculo adelante, a la derecha o atrás

  for (int i=0; i<=ultimoObstaculo; i++){
    //Busca si  la pose actual calza con la pose cuando se detectó el obstáculo. Uso un margen de tolerancia para la detección
    if ( (abs(poseActual[0]-datosSensores[i][0]) < unidadAvance) && 
         (abs(poseActual[1]-datosSensores[i][1]) < unidadAvance) && 
          datosSensores[i][2]==poseActual[2]){
            
        //Evalua el ángulo del obstáculo y lo simplifica a si hay obstáculo adelante, a la derecha o a la izquierda
        if (datosSensores[i][5] >= -45 && datosSensores[i][5] <= 45) {obstaculoAdelante=true;}
        if (datosSensores[i][5] > 55 && datosSensores[i][5] < 95) {obstaculoDerecha=true;}
        if (datosSensores[i][5] > -95 && datosSensores[i][5] < -55) {obstaculoIzquierda=true;} 
    }
  }
  Serial.print("ObsPeri=>  I:");
  Serial.print(obstaculoIzquierda);
  Serial.print("  A:");
  Serial.print(obstaculoAdelante);
  Serial.print("  D:");
  Serial.println(obstaculoDerecha);
  delay(10);
}

void AsignarDireccionRWD(){
//Función que escoge la dirección de giro para el robot en base al algoritmo Random Walk con Dirección
//Actualiza la variable global anguloGiro

    int diferenciaOrientacion = poseActual[2] - (int)direccionGlobal;
    int minRandom = 0;
    int maxRandom = 3;
    int incrementoPosible = 90;

    //En esta condición especial se asigna una nueva dirección global
    if (obstaculoAdelante && diferenciaOrientacion==0){ 
      direccionGlobal= (orientacionesRobot)(random(-1,3)*90); 
    }
    //Si hay un obstáculo o se está en una orientación diferente a la global, se restringen las opciones del aleatorio
    if (obstaculoIzquierda || diferenciaOrientacion == -90){ minRandom++; }
    if (obstaculoDerecha || diferenciaOrientacion == 90){ maxRandom--; }
    if (obstaculoAdelante || abs(diferenciaOrientacion)==180){
      incrementoPosible=180; //Un obstáculo adelante es un caso especial que implica girar a la derecha o izquierda (diferencia de 180°)
      maxRandom--;
    }
    if (obstaculoIzquierda && obstaculoAdelante && obstaculoDerecha){ //condición especial (callejón)
      anguloGiro=180;
      direccionGlobal= (orientacionesRobot)(random(-1,3)*90); 
    }else{
      //La ecuación del ángulo de giro toma en cuenta todas las restricciones anteriores y lo pasa a escala de grados (nomenclatura de orientación)
      anguloGiro = random(minRandom,maxRandom)*incrementoPosible-90;
    }

    Serial.print("DirGlobal: ");
    Serial.print(direccionGlobal);
    Serial.print("   Giro: ");
    Serial.println(anguloGiro);
    delay(10);
    
    //Reset de la simplificación sobre obstáculo en la pose actual
    obstaculoAdelante=false;
    obstaculoDerecha=false;
    obstaculoIzquierda=false;    
}


bool Giro(float grados){

    posActualRuedaDerecha= ConvDistAngular(contPulsosDerecha);
    int cicloTrabajoRuedaDerecha = ControlPosGiroRueda( grados, posActualRuedaDerecha, sumErrorGiroDer, errorAnteriorGiroDer );
    
    posActualRuedaIzquierda= ConvDistAngular(contPulsosIzquierda);
    int cicloTrabajoRuedaIzquierda = ControlPosGiroRueda( -grados, -posActualRuedaIzquierda, sumErrorGiroIzq, errorAnteriorGiroIzq);
    
//    Serial.print(cicloTrabajoRuedaDerecha);
//    Serial.print(",");
//    Serial.println(cicloTrabajoRuedaIzquierda);
  
    ConfiguraEscribePuenteH (cicloTrabajoRuedaDerecha, cicloTrabajoRuedaIzquierda);

    bool giroListo = EstadoEstacionario (cicloTrabajoRuedaDerecha, cicloTrabajoRuedaIzquierda, contCiclosEstacionarioGiro);

    return giroListo;

} 

float ConvDistAngular(float cantPulsos){
//Calcula la distancia angular (en grados) que se ha desplazado el robot. 
//Para robots diferenciales, en base a un giro sobre su eje (no arcos)
//Recibe la cantidad de pulsos de la rueda. Devuelve un ángulo en grados.

  float despAngular=cantPulsos/(distanciaCentroARueda*pulsosPorMilimetro)*180/PI; 
  return despAngular;
}

int ControlPosGiroRueda( float posRef, float posActual, float& sumErrorGiro, float& errorAnteriorGiro ){
//Funcion para implementar el control PID por posición en una rueda. 
//Se debe tener un muestreo en tiempos constantes, no aparecen las constantes de tiempo en la ecuación, sino que se integran con las constantes Ki y Kd
//Se recibe la posición de referencia, la posición actual (medida con los encoders), el error acumulado (por referencia), y el valor anterior del error (por referencia)
//Entrega el valor de ciclo de trabajo (PWM) que se enviará al motor CD

  float errorGiro= posRef-posActual; //se actualiza el error actual

  //se actualiza el error integral y se restringe en un rango, para no aumentar sin control
  //como son variables que se mantienen en ciclos futuros, se usan variables globales
  sumErrorGiro += errorGiro; 
  sumErrorGiro = constrain(sumErrorGiro,errorMinIntegral,errorMaxIntegral);

  //error derivativo (diferencial)
  float difErrorGiro= (errorGiro - errorAnteriorGiro);

  //ecuación de control PID
  float pidTermGiro= (KpGiro*errorGiro) + (KiGiro * sumErrorGiro) + (KdGiro*difErrorGiro);

  //Se limita los valores máximos y mínimos de la acción de control, para no saturarla
  pidTermGiro= constrain( int (pidTermGiro),limiteInferiorCicloTrabajoGiro,limiteSuperiorCicloTrabajoGiro); 

  //Restricción para manejar la zona muerta del PWM sobre la velocidad
  if (-cicloTrabajoMinimo < pidTermGiro && pidTermGiro < cicloTrabajoMinimo){
    pidTermGiro = 0; 
  }
  
  //actualiza el valor del error para el siguiente ciclo
  errorAnteriorGiro = errorGiro;
  
  return  pidTermGiro; 
}

void ConfiguraEscribePuenteH (int pwmRuedaDer, int pwmRuedaIzq){
  
//Determina los valores de pwm corregidos según la calibración
  int pwmRuedaIzqCorr=alfaIzq*pwmRuedaIzq+betaIzq; 
  int pwmRuedaDerCorr=alfaDer*pwmRuedaDer+betaDer;
  
//Determina si es giro, avance, o retroceso en base a los valores de PWM y configura los pines del Puente H  
  if (pwmRuedaDerCorr>=0 && pwmRuedaIzqCorr>=0){
    ConfiguracionAvanzar();
    analogWrite(PWMB, pwmRuedaDerCorr);
    analogWrite(PWMA, pwmRuedaIzqCorr);
    
  }else if (pwmRuedaDerCorr<0 && pwmRuedaIzqCorr<0){
    ConfiguracionRetroceder();
    analogWrite(PWMB, -pwmRuedaDerCorr);
    analogWrite(PWMA, -pwmRuedaIzqCorr);
    
  }else if (pwmRuedaDerCorr>0 && pwmRuedaIzqCorr<0){
    ConfiguracionGiroDerecho();
    analogWrite(PWMB, pwmRuedaDerCorr);
    analogWrite(PWMA, -pwmRuedaIzqCorr);
    
  }else if (pwmRuedaDerCorr<0 && pwmRuedaIzqCorr>0){
    ConfiguracionGiroIzquierdo();
    analogWrite(PWMB, -pwmRuedaDerCorr);
    analogWrite(PWMA, pwmRuedaIzqCorr);
  }
}

void ConfiguracionParar(){
  //Deshabilita las entradas del puente H, por lo que el carro se detiene
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, LOW); 
  ResetContadoresEncoders();
  delay(250);
}

void ConfiguracionAvanzar(){
  //Permite el avance del carro al poner las patillas correspondientes del puente H en alto
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, HIGH);
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, HIGH); 
}

void ConfiguracionRetroceder(){
  //Configura el carro para retroceder al poner las patillas correspondientes del puente H en alto
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, LOW); 
}

void ConfiguracionGiroDerecho(){
  //Configura las patillas del puente H para realizar un giro derecho
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, HIGH); 
}

void ConfiguracionGiroIzquierdo(){
  //Configura las patillas del puente H para realizar un giro izquierdo
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, HIGH);
  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, LOW); 
}

bool EstadoEstacionario (int pwmRuedaDer, int pwmRuedaIzq, int& contCiclosEstacionario){
//Función que revisa el estado de la acción de control y si se mantiene varios ciclos en cero, asume que ya está en el estado estacionario
//Recibe los ciclos de trabajo en cada rueda y una variable por referencia donde se almacenan cuantos ciclos seguidos se llevan
//Devuelve una variable que es TRUE si ya se alcanzó el estado estacionario

    bool estadoEstacionarioAlcanzado= false;
    
    if(pwmRuedaDer==0 && pwmRuedaIzq==0){
       contCiclosEstacionario++;
       if (contCiclosEstacionario > minCiclosEstacionario){
          //ResetContadoresEncoders();
          contCiclosEstacionario=0;
          estadoEstacionarioAlcanzado= true;
       }     
    }else{
      contCiclosEstacionario=0;
    }

  return estadoEstacionarioAlcanzado;
}

void ResetContadoresEncoders(){
//Realiza una inicialización de las variables que cuentan cantidad de pulsos y desplazamiento de la rueda

  contPulsosDerecha=0; 
  contPulsosIzquierda=0;

  posActualRuedaDerecha=0.0;
  posActualRuedaIzquierda=0.0;

  contPulsosDerPasado=0;
  contPulsosIzqPasado=0;
  
}

void PulsosRuedaDerechaC1(){
//Manejo de interrupción del canal C1 del encoder de la rueda derecha
  LecturaEncoder(ENC_DER_C1, ENC_DER_C2, estadoEncoderDer, contPulsosDerecha);
}

void PulsosRuedaDerechaC2(){
//Manejo de interrupción del canal C2 del encoder de la rueda derecha
  LecturaEncoder(ENC_DER_C1, ENC_DER_C2, estadoEncoderDer, contPulsosDerecha);
}

void PulsosRuedaIzquierdaC1(){
//Manejo de interrupción del canal C1 del encoder de la rueda izquierda
  LecturaEncoder(ENC_IZQ_C1, ENC_IZQ_C2, estadoEncoderIzq, contPulsosIzquierda);
}

void PulsosRuedaIzquierdaC2(){
//Manejo de interrupción del canal C2 del encoder de la rueda izquierda
  LecturaEncoder(ENC_IZQ_C1, ENC_IZQ_C2, estadoEncoderIzq, contPulsosIzquierda);
}

void LecturaEncoder(int entradaA, int entradaB, byte& state, int& contGiro){
//Función que determina si el motor está avanzando o retrocediendo en función del estado de las salidas del encoder y el estado anterior
//Modifica la variable contadora de pulsos de ese motor

  //se almacena el valor actual del estado
  byte statePrev = state;

  //lectura de los encoders
  int A = digitalRead(entradaA);
  int B = digitalRead(entradaB);

  //se define el nuevo estado
  if ((A==HIGH)&&(B==HIGH)) state = 1;
  if ((A==HIGH)&&(B==LOW)) state = 2;
  if ((A==LOW)&&(B==LOW)) state = 3;
  if ((A==LOW)&&(B==HIGH)) state = 4;

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


bool AvanzarDistancia(int distanciaDeseada){
//Avanza hacia adelante una distancia definida en mm a velocidad constante
//Devuelve true cuando alcanzó la distancia deseada

  velActualDerecha= calculaVelocidadRueda(contPulsosDerecha, contPulsosDerPasado);
  int cicloTrabajoRuedaDerecha = ControlVelocidadRueda(velRequerida, velActualDerecha, sumErrorVelDer, errorAnteriorVelDer);

  velActualIzquierda= -1.0 * calculaVelocidadRueda(contPulsosIzquierda, contPulsosIzqPasado); //como las ruedas están en espejo, la vel es negativa cuando avanza, por eso se invierte
  int cicloTrabajoRuedaIzquierda = ControlVelocidadRueda(velRequerida, velActualIzquierda, sumErrorVelIzq, errorAnteriorVelIzq);
   
//    Serial.print("Vel der: ");
//    Serial.print(velActualDerecha,5);
//    Serial.print(" ; Vel izq: ");
//    Serial.println(velActualIzquierda,5);
//
//    Serial.print("PWM der: ");
//    Serial.print(cicloTrabajoRuedaDerecha);
//    Serial.print(" ; PWM izq: ");
//    Serial.println(cicloTrabajoRuedaIzquierda);
  
  ConfiguraEscribePuenteH (cicloTrabajoRuedaDerecha, cicloTrabajoRuedaIzquierda);

  float distanciaAvanzada= calculaDistanciaLinealRecorrida();
  
  bool avanceListo = false; 
  if (distanciaAvanzada >= distanciaDeseada) {
    avanceListo = true; 
    //ResetContadoresEncoders();
  }
  
  return avanceListo;
}

void AvanzarIndefinido(){
//Avanza hacia adelante indefinidamente
//Luego de utilizarse, se debe llamar a ResetContadoresEncoders()

  velActualDerecha= calculaVelocidadRueda(contPulsosDerecha, contPulsosDerPasado);
  int cicloTrabajoRuedaDerecha = ControlVelocidadRueda(velRequerida, velActualDerecha, sumErrorVelDer, errorAnteriorVelDer);

  velActualIzquierda= -1.0 * calculaVelocidadRueda(contPulsosIzquierda, contPulsosIzqPasado); //como las ruedas están en espejo, la vel es negativa cuando avanza, por eso se invierte
  int cicloTrabajoRuedaIzquierda = ControlVelocidadRueda(velRequerida, velActualIzquierda, sumErrorVelIzq, errorAnteriorVelIzq);
  
  ConfiguraEscribePuenteH (cicloTrabajoRuedaDerecha, cicloTrabajoRuedaIzquierda);

}

float calculaVelocidadRueda(int& contPulsos, int& contPulsosPasado){
//Función que calcula la velocidad de una rueda en base a la cantidad de pulsos del encoder y el tiempo de muestreo

    float velActual= ((contPulsos-contPulsosPasado) / pulsosPorMilimetro) / ((float)tiempoMuestreo/conversionMicroSaSDiv); //velocidad en mm por s
    contPulsosPasado= contPulsos;
    
    return velActual;
}

float calculaDistanciaLinealRecorrida(){
//Función que realiza el cálculo de la distancia lineal recorrida por cada rueda
//Devuelve el promedio de las distancias

  float distLinealRuedaDerecha= (float)contPulsosDerecha / pulsosPorMilimetro;
  float distLinealRuedaIzquierda= - (float)contPulsosIzquierda / pulsosPorMilimetro;
  float DistanciaLineal= (distLinealRuedaDerecha+distLinealRuedaIzquierda) /2.0;

  return DistanciaLineal;
}

int ControlVelocidadRueda( float velRef, float velActual, float& sumErrorVel, float& errorAnteriorVel ){
//Funcion para implementar el control PID por velocidad en una rueda. 
//Se debe tener un muestreo en tiempos constantes, no aparecen las constantes de tiempo en la ecuación, sino que se integran con las constantes Ki y Kd
//Se recibe la posición de referencia, la posición actual (medida con los encoders), el error acumulado (por referencia), y el valor anterior del error (por referencia)
//Entrega el valor de ciclo de trabajo (PWM) que se enviará al motor CD

  float errorVel= velRef-velActual; //se actualiza el error actual
   
  //se actualiza el error integral y se restringe en un rango, para no aumentar sin control
  //como son variables que se mantienen en ciclos futuros, se usan variables globales
  sumErrorVel += errorVel; 
  sumErrorVel = constrain(sumErrorVel,errorMinIntegral,errorMaxIntegral);

  //error derivativo (diferencial)
  float difErrorVel= (errorVel - errorAnteriorVel);

  //ecuación de control PID
  float pidTermVel= (KpVel*errorVel) + (KiVel * sumErrorVel) + (KdVel*difErrorVel);

  //Se limita los valores máximos y mínimos de la acción de control, para no saturarla
  pidTermVel= constrain( (int)pidTermVel,limiteInferiorCicloTrabajoVelocidad,limiteSuperiorCicloTrabajoVelocidad); 

//  //Restricción para manejar la zona muerta del PWM sobre la velocidad
//  if (-cicloTrabajoMinimo < pidTermVel && pidTermVel < cicloTrabajoMinimo){
//    pidTermVel = 0; 
//  }
      
  //actualiza el valor del error para el siguiente ciclo
  errorAnteriorVel = errorVel;
  
  return  ((int)pidTermVel); 
}

void inicializaMagnet(){
//Funcion que establece la comunicacion con el magnetometro, lo setea para una frecuencia de muestreo de 200 Hz
//y en modo de medición continua
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
//Funcion que extrae los datos crudos del magnetometro, pasa los parametros por referencia y carga los valores de calibracion
//desde la memoria eeprom, adicionalmente usa un filtro para la toma de datos (media movil)
  myWire.beginTransmission(addr);
  myWire.write(0x00); //start with register 3.
  myWire.endTransmission();

  //Pide 6 bytes del registro del magnetometro
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


float leerDatoFloat(int dirPagInicial){
//Funcion que lee datos flotantes de la memoria eeprom, pasa como parametro la posicion inicial
//de memoria para comenzar a leer. Se leen los primeros 8 bits que serian LSB y luego los 8 bits correspondientes al MSB
//luego se leen los 8 bits correspondientes a la parte decimal y por ultimo 8 bits que determinan el signo (==0 seria positivo e ==1 seria negativo)

    byte signo; //signo del numero, 1 para negativo, 0 para positivo
    byte varr1; //primeros 8 bits para guardar en la EEPROM (LSB)
    byte varr2; //ultimo 8 bits para guardar en la EEPROM (MSB)
    byte varrDec; //parte decimal para guardar en la EEPROM
    float dato;
    //Lee 4 posiciones de memoria a partir de la ingresada como parametro
    varr1=eepromLectura(dirEEPROM,dirPagInicial);
    varr2=eepromLectura(dirEEPROM,dirPagInicial+1);
    varrDec=eepromLectura(dirEEPROM,dirPagInicial+2);
    signo=eepromLectura(dirEEPROM,dirPagInicial+3);
    //Agrega el signo
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
//Funcion que "arma" un numero flotante a partir de sus 8 bits LSB, sus 8 bits MSB y 8 bits para la
//parte decimal del numero, devuelve el numero reconstruido como flotante
    int piv1,piv2;
    float nuevo,decs;
    piv1=LSB;
    piv2=MSB<<8;
    nuevo=piv1|piv2;
    decs=float(dec);
    nuevo=nuevo+decs/100;
    return nuevo;
}

byte eepromLectura(int dir, int dirPag) {
//Funcion que lee datos de la memoria eeprom, pasa como parametros la direccion del dispositivo (hexadecimal)
//y la direccion de pagina donde se desea leer, devuelve el dato de 8 bits contenido en esa direccion
  myWire.beginTransmission(dir);
  myWire.write(dirPag);
  myWire.endTransmission();
  myWire.requestFrom(dir, 1);
  if(myWire.available())
    return myWire.read();
  else
    return 0xFF;
}
