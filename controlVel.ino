//motor Izquierda conectado a A1N1 y AIN2
//motor Derecha conectado a BIB1 y BIN2
//PWM min para operación es de 27
#define PWMA  10 //Speed control
#define AIN1  11 //Direction
#define AIN2  12
#define PWMB  9
#define BIN1  6
#define BIN2  5

const int tiempoMuestreo=100;
const int conversionSegundosTiempoMuestreo=(60*1000)/tiempoMuestreo;
const int pulsosPorRev=206;
const int limiteSuperiorCicloTrabajo=255;
const int limiteInferiorCicloTrabajo=0;

int contPulsosDerecha=0;
int contPulsosIzquierda=0;
int vMD=100;//valor bin equivalente al duty cicle motor derecho
int vMI=100;// valor bin equivalente al duty cicle motor izquierdo
int tiempoActual=0;
int velActualDerecha=0;  //dato en RPM
int velActualIzquierda=0; //dato en RPM
const int velRequerida=150; //unidades RPM
const float Kp=0.5; //constante control proporcional
const float Kd=0.6; //constante control derivativo
const float Ki=0.1; //constante control integral
const int errorMinIntegral=-4000;
const int errorMaxIntegral=4000;

void setup() {
  // put your setup code here, to run once:
  pinMode(PWMA, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  attachInterrupt(A0, PulsosRuedaDerecha, RISING);  //conectado el contador C1 rueda derecha
  //attachInterrupt(A1, DC2, CHANGE);
  attachInterrupt(A2, PulsosRuedaIzquierda, RISING); //conectado el contador C1 rueda izquierda
  //attachInterrupt(A3, IC2, CHANGE);
  Serial.begin(9600);
  tiempoActual=millis();
}
int ControlVelocidadDerecha(int command, int velRef, int velActual){
  //Funcion para implementar el control PID de la velocidad de la rueda Derecha
  float pidTermDer=0;
  int errorDer=0;
  static int errorAnteriorDer=0;
  static int sumErrorDer=0;
  errorDer=abs(velRef)-abs(velActual);
  pidTermDer= (Kp*errorDer)+Ki*sumErrorDer+(Kd*(errorDer-errorAnteriorDer));
  sumErrorDer+= errorDer;
  sumErrorDer=constrain(sumErrorDer,errorMinIntegral,errorMaxIntegral);
  errorAnteriorDer=errorDer;
  return constrain(command + int (pidTermDer),limiteInferiorCicloTrabajo,limiteSuperiorCicloTrabajo);  
}
int ControlVelocidadIzquierda(int command, int velRef, int velActual){
  //Funcion que permite implementar el control PID de la rueda izquierda
  float pidTermIzq=0;
  int errorIzq=0;
  static int errorAnteriorIzq=0;
  static int sumErrorIzq=0;
  errorIzq=abs(velRef)-abs(velActual);
  pidTermIzq= (Kp*errorIzq)+Ki*sumErrorIzq+(Kd*(errorIzq-errorAnteriorIzq));
  sumErrorIzq+= errorIzq;
  sumErrorIzq=constrain(sumErrorIzq,errorMinIntegral,errorMaxIntegral);
  errorAnteriorIzq=errorIzq;
  return constrain(command + int (pidTermIzq),limiteInferiorCicloTrabajo,limiteSuperiorCicloTrabajo);  
}



void Avanzar(){
  //Permite el avance del carro al poner las patillas correspondientes del puente H en alto
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, HIGH);
  vMI=ControlVelocidadIzquierda(vMI,velRequerida,velActualIzquierda);
  analogWrite(PWMA, vMI);
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, HIGH); 
  vMD=ControlVelocidadDerecha(vMD,velActualIzquierda,velActualDerecha);
  analogWrite(PWMB, vMD); 
}

void Velocidad(){
  //Esta función calcula las rpm en cada una de las ruedas 
    velActualDerecha=contPulsosDerecha*conversionSegundosTiempoMuestreo/pulsosPorRev; //conversión de pulsos/s a RPM
    velActualIzquierda=contPulsosIzquierda*conversionSegundosTiempoMuestreo/pulsosPorRev;
    contPulsosDerecha=0;
    contPulsosIzquierda=0;
}

void loop() {
  if((millis()-tiempoActual)>=tiempoMuestreo){
    tiempoActual=millis();
    Velocidad();
    Avanzar();
  }
  Serial.print("Velocidad rueda derecha=");
  Serial.print(velActualDerecha);
  Serial.print(" Velocidad izquierda= ");
  Serial.println(velActualIzquierda);
}


void PulsosRuedaDerecha(){
  //Esta funcion lleva la cuenta de los pulsos del encoder C1 en la rueda derecha
    contPulsosDerecha++; //Utilicado para el cálculo de velocidad
}

void PulsosRuedaIzquierda(){
  //Esta funcion lleva la cuenta de los pulsos del enconder C1 en la rueda izquierda
    contPulsosIzquierda++; //Utilizado para el cálculo de velocidad 
}


