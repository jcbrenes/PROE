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
const int correccionSegundosTiempoMuestreo=(60*1000)/tiempoMuestreo;
const int pulsosPorRev=206;
const int limiteSuperiorCicloTrabajo=255;
const int limiteInferiorCicloTrabajo=0;

int contPulsosDerecha=0;
int contPulsosIzquierda=0;
int vMD=100;//valor bin equivalente al duty cicle motor derecho
int vMI=100;// valor bin equivalente al duty cicle motor izquierdo
int tiempoActual=0;
int velActualDerecha=0;
int velActualIzquierda=0;
const int velRequerida=150; //rpm
int setPoint=100;
float Kp=0.5;
float Kd=0.6;
float Ki=0.1;

int conteoVelocidadActualDerecha=0;
int errorMinIntegral=-4000;
int errorMaxIntegral=4000;

void setup() {
  // put your setup code here, to run once:
  pinMode(PWMA, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  attachInterrupt(A0, PulsosRuedaDerecha, RISING);
  //attachInterrupt(A1, DC2, CHANGE);
  attachInterrupt(A2, PulsosRuedaIzquierda, RISING);
  //attachInterrupt(A3, IC2, CHANGE);
  Serial.begin(9600);
  tiempoActual=millis();
}
int ControlVelocidadDerecha(int command, int velRef, int velActual){
  float pidTerm=0;
  int error=0;
  static int last_error=0;
  static int sum_error=0;
  error=abs(velRef)-abs(velActual);
  pidTerm= (Kp*error)+Ki*sum_error+(Kd*(error-last_error));
  sum_error+= error;
  sum_error=constrain(sum_error,errorMinIntegral,errorMaxIntegral);
  last_error=error;
  return constrain(command + int (pidTerm),limiteInferiorCicloTrabajo,limiteSuperiorCicloTrabajo);  
}
int ControlVelocidadIzquierda(int command, int velRef, int velActual){
  float pidTerm2=0;
  int error2=0;
  static int last_error2=0;
  static int sum_error2=0;
  error2=abs(velRef)-abs(velActual);
  pidTerm2= (Kp*error2)+Ki*sum_error2+(Kd*(error2-last_error2));
  sum_error2+= error2;
  sum_error2=constrain(sum_error2,errorMinIntegral,errorMaxIntegral);
  last_error2=error2;
  return constrain(command + int (pidTerm2),limiteInferiorCicloTrabajo,limiteSuperiorCicloTrabajo);  
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
    velActualDerecha=contPulsosDerecha*correccionSegundosTiempoMuestreo/pulsosPorRev;
    velActualIzquierda=contPulsosIzquierda*correccionSegundosTiempoMuestreo/pulsosPorRev;
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


