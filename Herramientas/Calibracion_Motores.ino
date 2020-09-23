//Este código se utiliza para calibrar 2 motores DC
//Da como resultado valores númericos de calibración que deben ser apuntados e implementados en el código principal
//La "condición" mencionada se refiere a que el PWM que se calibre no debe exceder los 255

const int tiempoMuestreo=10000; //unidades: micro segundos
const float pulsosPorRev=206.0; //cantidad de pulsos de una única salida
const int factorEncoder=4; //cantidad de tipos de pulsos que se están detectando (juego entre las 2 salidas del encoder)
const float circunferenciaRueda=139.5;//Circunferencia de la rueda = 139.5mm 
const float pulsosPorMilimetro=((float)factorEncoder*pulsosPorRev)/circunferenciaRueda; 
const float distanciaCentroARueda=65.0;// Radio de giro del carro, es la distancia en mm entre el centro y una rueda. Estaba a 65.75
const float conversionMicroSaMin=1/(60 * 1000000);// factor de conversion microsegundo (unidades del tiempo muestreo) a minuto
const float conversionMicroSaSDiv=1000000;// factor de conversion microsegundo (unidades del tiempo muestreo) a segundo
const float tiempoMuestreoS= (float)tiempoMuestreo/conversionMicroSaSDiv;

const int ENC_DER_C1 =  A0; //Encoders rueda derecha
const int ENC_DER_C2 =  A1;
const int ENC_IZQ_C1 =  A2; //Encoders rueda izquierda
const int ENC_IZQ_C2 =  A3;

long tiempoActual=0;
//contadores de pulsos del encoder, muy importantes
int contPulsosDerecha=0;
int contPulsosIzquierda=0;
int contPulsosDerPasado=0;
int contPulsosIzqPasado=0;

float posActualRuedaDerecha=0.0;
float posActualRuedaIzquierda=0.0;

float velActualDerecha=0.0;
float velActualIzquierda=0.0;

byte estadoEncoderDer=1; 
byte estadoEncoderIzq=1;

const int PWMA = 10; //Control velocidad izquierdo
const int AIN1 = 11; //Dirección 1 rueda izquierda 
const int AIN2 = 12; //Dirección 2 rueda izquierda 
const int PWMB = 9;  //Control velocidad derecha
const int BIN1 = 6;  //Dirección 1 rueda derecha
const int BIN2 = 5;  //Dirección 2 rueda derecha

float pendienteIzq=0; //pendiente del motor izquierdo
float bIzq=0;         //coeficiente b del motor izquierdo
float pendienteDer=0; //pendiente del motor derecho
float bDer=0;        //coeficiente b del motor derecho


void setup() {
  
  pinMode(PWMA, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(13,OUTPUT);
  attachInterrupt(ENC_DER_C1, PulsosRuedaDerechaC1,CHANGE);  //conectado el contador C1 rueda derecha
  attachInterrupt(ENC_DER_C2, PulsosRuedaDerechaC2,CHANGE); 
  attachInterrupt(ENC_IZQ_C1, PulsosRuedaIzquierdaC1,CHANGE);  //conectado el contador C1 rueda izquierda
  attachInterrupt(ENC_IZQ_C2, PulsosRuedaIzquierdaC2,CHANGE);
  Serial.begin(9600);
  tiempoActual=micros();
  delay(3000);
}

void loop() {
  int pwmMotorIzquierdo=20; //contador de PWM para motor izquierdo
  int pwmMotorDerecho=20; //contador de PWM para motor derecho
  const int tiempoDelay=1000; 
  const int numDatos=48; //número de muestras 20 al 255
  const int sumPWM=6600;  //sumatoria de todo los pwm
  const int sumCuadPWM=1137800 ; //sumatoria de todo los PWM al cuadrado
  const int valorMaxPWM=255;
  float velRuedaIzq=0;
  float velRuedaDer=0;
  float sumVelMotorIzq=0; //sumatoria de velocidad de motor izquierdo
  float sumVelPwmIzq=0; //sumatoria de pwm por velocidad en motor izquierdo
  float sumVelMotorDer=0; //sumatoria de velocidades del motor derecho
  float sumVelPwmDer=0; //sumatoria de pwm por velocidad motor derecho
  long delayCalibracion=0;
  float alfa=0; //valor de calibracion multiplicativo
  float beta=0; //valor de calibracion aditivo
  int maxPWM=0; // Maximo PWM para el motor seleccionado como master en los 2 casos especiales
  digitalWrite(13,HIGH);
  ResetContadoresEncoders();
  while(pwmMotorIzquierdo<=valorMaxPWM){  
    delayCalibracion=millis();
    ConfiguraEscribePuenteH(pwmMotorDerecho,pwmMotorIzquierdo);
    while (millis()<delayCalibracion+tiempoDelay){} //Espera 1 segundo mientras se estabilizan los motores
    velRuedaIzq=calculaVelocidadRueda(contPulsosIzquierda,contPulsosIzqPasado); //obtiene velocidad en mm/s del motor izquierdo
    velRuedaDer=calculaVelocidadRueda(contPulsosDerecha,contPulsosDerPasado); //obtiene velocidad en mm/s del motor derecho
    sumVelMotorIzq=sumVelMotorIzq+ velRuedaIzq; 
    sumVelMotorDer=sumVelMotorDer+ velRuedaIzq;
    sumVelPwmIzq=sumVelPwmIzq + velRuedaIzq*pwmMotorIzquierdo;
    sumVelPwmDer=sumVelPwmDer + velRuedaDer*pwmMotorDerecho;
    pwmMotorIzquierdo=pwmMotorIzquierdo+5;
    pwmMotorDerecho=pwmMotorDerecho+5;
  }
  ConfiguraEscribePuenteH(0,0);
  //Mediante el método de mínimos cuadrados calcula la pendiente y el coeficiente b para cada motor
  pendienteIzq=(sumVelPwmIzq-((sumPWM*sumVelMotorIzq)/numDatos))/(sumCuadPWM-(sumPWM^2/numDatos)); 
  pendienteDer=(sumVelPwmDer-((sumPWM*sumVelMotorDer)/numDatos))/(sumCuadPWM-(sumPWM^2/numDatos));
  bIzq=(sumVelMotorIzq/numDatos)-pendienteIzq*(sumPWM/numDatos);
  bDer=(sumVelMotorDer/numDatos)-pendienteDer*(sumPWM/numDatos);
  //Según los resultados obtenidos se selecciona un master y se calculan los valores de calibración 
  if (NuevoPWM(valorMaxPWM,pendienteIzq,pendienteDer,bIzq,bDer)<=valorMaxPWM){ //Caso en donde el motor izquierdo es master y cumple la condición 
      alfa = pendienteIzq/pendienteDer;
      beta=(bIzq-bDer)/pendienteDer;
      Serial.println("---------- Valores de calibración: aplicar PWMmotorDerecho= Alfa*PWMmotorIzquierdo + Beta ---------");
      Serial.print("Alfa:  "); Serial.print(alfa); Serial.print(" Beta:  "); Serial.println(beta);
    }
   else if (NuevoPWM(valorMaxPWM,pendienteDer,pendienteIzq,bDer,bIzq)<=valorMaxPWM){ //Caso en donde el motor derecho es master y cumple la condición
      alfa = pendienteDer/pendienteIzq;
      beta=(bDer-bIzq)/pendienteIzq;
      Serial.println("---------- Valores de calibración:  aplicar PWMmotorIzquierdo= Alfa*PWMmotorDerecho + Beta ---------");
      Serial.print("Alfa:  "); Serial.print(alfa); Serial.print(" Beta:  "); Serial.println(beta);
    } 
   else if (CondicionMaster(pendienteIzq,bIzq,pendienteDer,bDer)){ //Derecho es master y se ajusta su valor maximo de PWM en el PID
      alfa = pendienteDer/pendienteIzq;
      beta=(bDer-bIzq)/pendienteIzq;
      maxPWM=int((valorMaxPWM-beta)/alfa); 
      Serial.println("---------- Valores de calibración:  aplicar PWMmotorIzquierdo= Alfa*PWMmotorDerecho + Beta ---------");
      Serial.print("Alfa:  "); Serial.print(alfa); Serial.print(" Beta:  "); Serial.println(beta);
      Serial.print("Ajustar valor máximo de PWM para motor derecho en "); Serial.println(maxPWM);
     } 
   else { //Izquierdo es master y se ajusta su valor maximo de PWM en el PID
      alfa = pendienteIzq/pendienteDer;
      beta=(bIzq-bDer)/pendienteDer;
      maxPWM=int((valorMaxPWM-beta)/alfa); 
      Serial.println("---------- Valores de calibración: aplicar PWMmotorDerecho= Alfa*PWMmotorIzquierdo + Beta ---------");
      Serial.print("Alfa:  "); Serial.print(alfa); Serial.print(" Beta:  "); Serial.println(beta);
      Serial.print("Ajustar valor máximo de PWM para motor izquierdo en "); Serial.println(maxPWM);
    }
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

float calculaVelocidadRueda(int& contPulsos, int& contPulsosPasado){
//Función que calcula la velocidad de una rueda en base a la cantidad de pulsos del encoder y el tiempo de muestreo

    float velActual= ((contPulsos-contPulsosPasado) / pulsosPorMilimetro) / ((float)tiempoMuestreo/conversionMicroSaSDiv); //velocidad en mm por s
    contPulsosPasado= contPulsos;
    
    return velActual;
}

void ConfiguraEscribePuenteH (int pwmRuedaDer, int pwmRuedaIzq){
//Determina si es giro, avance, o retroceso en base a los valores de PWM y configura los pines del Puente H

 
  if (pwmRuedaDer>=0 && pwmRuedaIzq>=0){
    ConfiguracionAvanzar();
    analogWrite(PWMB, pwmRuedaDer);
    analogWrite(PWMA, pwmRuedaIzq);
    
  }else if (pwmRuedaDer<0 && pwmRuedaIzq<0){
    ConfiguracionRetroceder();
    analogWrite(PWMB, -pwmRuedaDer);
    analogWrite(PWMA, -pwmRuedaIzq);
    
  }else if (pwmRuedaDer>0 && pwmRuedaIzq<0){
    ConfiguracionGiroDerecho();
    analogWrite(PWMB, pwmRuedaDer);
    analogWrite(PWMA, -pwmRuedaIzq);
    
  }else if (pwmRuedaDer<0 && pwmRuedaIzq>0){
    ConfiguracionGiroIzquierdo();
    analogWrite(PWMB, -pwmRuedaDer);
    analogWrite(PWMA, pwmRuedaIzq);
  }
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

bool CondicionMaster(float mIzq, float bIzq, float mDer, float bDer){
//Función que determina cual motor es mas apto para ser el master
//si retorna true el derecho es master y false para el izquierdo 
  if (NuevoPWM(255,mIzq,mDer,bIzq,bDer)>NuevoPWM(255,mDer,mIzq,bDer,bIzq)){
          return true;}
  else {return false; }
}

int NuevoPWM(int pwm, float mMaster, float mSlave, float bMaster, float bSlave){
//Funcion que calcula el PWM modificado segun se escoja el master y slave
  int nuevoPWM=pwm*(mMaster/mSlave)+(bMaster-bSlave)/mSlave;
  return nuevoPWM; 
}
