#define PWMA  10 //Speed control
#define AIN1  11 //Direction
#define AIN2  12
#define PWMB  9
#define BIN1  6
#define BIN2  5

const long tiempoMuestreo=5000;
const float pulsosPorRev=206.0;
const int factorEncoder=4;
const int limiteSuperiorCicloTrabajo=255;
const int limiteInferiorCicloTrabajo=0;
const float avanceLineal=139.5;//139.5
const float pulsosPorMilimetro=(factorEncoder*pulsosPorRev)/avanceLineal; //avance lineal 139.5mm/rev dividido por 206*2 pulsos.

const float KpVelocidad=0.5; //constante control proporcional
const float KdVelocidad=0.1; //constante control derivativo
const float KiVelocidad=0.01; //constante control integral
//constantes para control de giro
const float KpGiro=20; //constante control proporcional
const float KdGiro=2;//constante control derivativo
const float KiGiro=0.08; //constante control integral
const int errorMinIntegral=-100;
const int errorMaxIntegral=100;
const int limiteSuperiorCicloTrabajoGiro=60;
const int limiteInferiorCicloTrabajoGiro=-60;
const float radioGiro=65.0;// estaba a 65.75
const float errorGiroMax=0.5;
const float errorGiroPermitido=errorGiroMax/2;


int contGiroDer=0;
int contGiroIzq=0;
float pidTermGiroDer=0;
float errorGiroDer=0;
float errorAnteriorGiroDer=0;
float sumErrorGiroDer=0;

float pidTermGiroIzq=0;
float errorGiroIzq=0;
float errorAnteriorGiroIzq=0;
float sumErrorGiroIzq=0;
float posActualDerecha=0.0;
float posActualIzquierda=0.0;
float despAngular=0.0;

float contPulsosDerecha=0;
float contPulsosIzquierda=0;
int vMD=25;//valor bin proporcional al duty cicle motor derecho
int vMI=25;// valor bin proporcional al duty cicle motor izquierdo
long tiempoActual=0;

//Variables para maquina de estados
bool giro=false;
bool avance=false;
long contTemp=0;

//Variables para interrupcion encoders
boolean A,B,C,D;
byte state,state2;
byte statep,statep2;


void setup() {
  // put your setup code here, to run once:
  pinMode(PWMA, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(13,OUTPUT);
  attachInterrupt(A0, PulsosRuedaDerechaC1,CHANGE);  //conectado el contador C1 rueda derecha
  attachInterrupt(A1, PulsosRuedaDerechaC2,CHANGE); //conectado el contador C1 rueda izquierda
  attachInterrupt(A2, PulsosRuedaIzquierdaC1,CHANGE);
  attachInterrupt(A3, PulsosRuedaIzquierdaC2,CHANGE);
  Serial.begin(9600);
  tiempoActual=micros();
}

void loop(){
  if(contTemp>5000){
    contTemp=0;
  }
  if((micros()-tiempoActual)>=tiempoMuestreo){
    tiempoActual=micros();
    
    contTemp++;
    giro=true;
    if(contTemp<2500){
    Giro(-90.0);  
    }
    else{
      Giro(0.0);
    }
 
  }
  
}
void Giro(float grados){
  //la función correspondiente realiza el giro a la derecha
  if(giro==true){
    posActualDerecha=ConvDespAngular(contGiroDer);
    vMD=ControlPosGiroRuedaDerecha(grados,posActualDerecha);
    posActualIzquierda=ConvDespAngular(contGiroIzq);
    vMI=ControlPosGiroRuedaIzquierda(grados,posActualIzquierda);
    Serial.print(vMD);
    Serial.print(",");
    Serial.println(vMI);
  
    if(vMD>0){
      digitalWrite(BIN1, LOW);
      digitalWrite(BIN2, HIGH); 
      analogWrite(PWMB, vMD);
    }
    else if(vMD<0){
      digitalWrite(BIN1, HIGH);
      digitalWrite(BIN2, LOW); 
      analogWrite(PWMB, -vMD);
    }
    else if(vMD==0){
       digitalWrite(BIN1, LOW);
       digitalWrite(BIN2, LOW);
       analogWrite(PWMB,0);
    }
    
    if(vMI>0){
      digitalWrite(AIN1, HIGH);
      digitalWrite(AIN2, LOW); 
      analogWrite(PWMA, vMI);
    }
   else if(vMI<0){
      digitalWrite(AIN1, LOW);
      digitalWrite(AIN2, HIGH); 
      analogWrite(PWMA, -vMI);
  }
   else if(vMI==0){
      digitalWrite(AIN1, LOW);
      digitalWrite(AIN2, LOW);
      analogWrite(PWMA,0);
  }
    if(vMD==0 && vMI==0){
      digitalWrite(13,HIGH);
    }
    else{
      digitalWrite(13,LOW);
    }
}
} 

int ControlPosGiroRuedaDerecha( float posRef, float posActual){
  //Funcion para implementar el control PID de la velocidad de la rueda Derecha para el giro
  errorGiroDer=posRef-posActual;
  sumErrorGiroDer+= errorGiroDer;
  sumErrorGiroDer=constrain(sumErrorGiroDer,errorMinIntegral,errorMaxIntegral);
  pidTermGiroDer= (KpGiro*errorGiroDer)+KiGiro*sumErrorGiroDer+(KdGiro*(errorGiroDer-errorAnteriorGiroDer));
  errorAnteriorGiroDer=errorGiroDer;
  if(errorGiroDer>-errorGiroPermitido && errorGiroDer<errorGiroPermitido){
    pidTermGiroDer=0;
  }
  if(pidTermGiroDer>0){
    pidTermGiroDer=map(pidTermGiroDer,0,55,25,limiteSuperiorCicloTrabajoGiro);
  }
  
  if(pidTermGiroDer<0){
    pidTermGiroDer=map(pidTermGiroDer,-55,0,limiteInferiorCicloTrabajoGiro,-25);
  }
  return constrain( int (pidTermGiroDer),limiteInferiorCicloTrabajoGiro,limiteSuperiorCicloTrabajoGiro);  
}
int ControlPosGiroRuedaIzquierda( float posRefI, float posActualI){
  //Funcion para implementar el control PID de la velocidad de la rueda Derecha para el giro
  errorGiroIzq=posRefI-posActualI;
  sumErrorGiroIzq+= errorGiroIzq;
  sumErrorGiroIzq=constrain(sumErrorGiroIzq,errorMinIntegral,errorMaxIntegral);
  pidTermGiroIzq= (KpGiro*errorGiroIzq)+KiGiro*sumErrorGiroIzq+(KdGiro*(errorGiroIzq-errorAnteriorGiroIzq));
  errorAnteriorGiroIzq=errorGiroIzq;
  if(errorGiroIzq>-errorGiroPermitido && errorGiroIzq<errorGiroPermitido){
    pidTermGiroIzq=0;
  }
   if(pidTermGiroIzq>0){
    pidTermGiroIzq=map(pidTermGiroIzq,0,55,25,limiteSuperiorCicloTrabajoGiro);
  }
  
  if(pidTermGiroIzq<0){
    pidTermGiroIzq=map(pidTermGiroIzq,-55,0,limiteInferiorCicloTrabajoGiro,-25);
  }
  return constrain( int (pidTermGiroIzq),limiteInferiorCicloTrabajoGiro,limiteSuperiorCicloTrabajoGiro);  
}

float ConvDespAngular(float pulsos){
   despAngular=(pulsos*avanceLineal*180)/(radioGiro*factorEncoder*pulsosPorRev*3.141592);
  return despAngular;
}

void PulsosRuedaDerechaC1(){
  EncoderDerecho();
}

void PulsosRuedaDerechaC2(){
  EncoderDerecho();
}
void EncoderDerecho(){
  A = digitalRead(A0);
  B = digitalRead(A1);

  if ((A==HIGH)&&(B==HIGH)) state = 1;
  if ((A==HIGH)&&(B==LOW)) state = 2;
  if ((A==LOW)&&(B==LOW)) state = 3;
  if((A==LOW)&&(B==HIGH)) state = 4;
  switch (state)
  {
    case 1:
    {
      if (statep == 2) contGiroDer--;
      if (statep == 4) contGiroDer++;
      break;
    }
    case 2:
    {
      if (statep == 1) contGiroDer++;
      if (statep == 3) contGiroDer--;
      break;
    }
    case 3:
    {
      if (statep == 2) contGiroDer++;
      if (statep == 4) contGiroDer--;
      break;
    }
    default:
    {
      if (statep == 1) contGiroDer--;
      if (statep == 3) contGiroDer++;
    }
  }
  statep = state;
}
void PulsosRuedaIzquierdaC1(){
 EncoderIzquierdo();

}

void PulsosRuedaIzquierdaC2(){
  EncoderIzquierdo();
}
void EncoderIzquierdo(){
  C = digitalRead(A2);
  D = digitalRead(A3);

  if ((C==HIGH)&&(D==HIGH)) state2 = 1;
  if ((C==HIGH)&&(D==LOW)) state2 = 2;
  if ((C==LOW)&&(D==LOW)) state2 = 3;
  if((C==LOW)&&(D==HIGH)) state2 = 4;
  switch (state2)
  {
    case 1:
    {
      if (statep2 == 2) contGiroIzq--;
      if (statep2 == 4) contGiroIzq++;
      break;
    }
    case 2:
    {
      if (statep2 == 1) contGiroIzq++;
      if (statep2 == 3) contGiroIzq--;
      break;
    }
    case 3:
    {
      if (statep2 == 2) contGiroIzq++;
      if (statep2 == 4) contGiroIzq--;
      break;
    }
    default:
    {
      if (statep2 == 1) contGiroIzq--;
      if (statep2 == 3) contGiroIzq++;
    }
  }
  statep2 = state2;
}




