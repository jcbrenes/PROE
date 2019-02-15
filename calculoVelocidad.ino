//motor A connected between A01 and A02
//motor B connected between B01 and B02
//PWM min para operación es de 27
int PWMA = 10; //Speed control
int AIN1 = 11; //Direction
int AIN2 = 12; //Direction
int PWMB=9 ;
int BIN1=6;
int BIN2=5;
int conteoDePulsosDerecha=0;
int conteoDePulsosIzquierda=0;
int vMD=100;//valor bin equivalente al duty cicle motor derecho
int vMI=100;// valor bin equivalente al duty cicle motor izquierdo
int tiempoActual=0;
int velDerecha=0;
int velIzquierda=0;
const int velRef=500; //pulsos por segundo
int error=0;
int setPoint=100;
int Kp=1;
const int tiempoMuestreo=100;
const int correccionSegundosDeTiempoMuestreo=10;
const int limiteSuperiorCicloTrabajo=255;
const int limiteInferiorCicloTrabajo=0;
int conteoVelocidadActualDerecha=0;

void setup() {
  // put your setup code here, to run once:
  pinMode(PWMA, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  attachInterrupt(A0, ContarPulsosRuedaDerecha, RISING);
  //attachInterrupt(A1, DC2, CHANGE);
  attachInterrupt(A2, ContarPulsosRuedaIzquierda, RISING);
  //attachInterrupt(A3, IC2, CHANGE);
  Serial.begin(9600);
  tiempoActual=millis();
}


void Avanzar(){
  //Permite el avance del carro al poner las patillas correspondientes del puente H en alto
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, HIGH);
  analogWrite(PWMA, vMI);
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, HIGH);
  analogWrite(PWMB, vMD);
  
}
void Velocidad(){
  //Esta función calcula los pulsos por segundo en cada una de las ruedas 
  if((millis()-tiempoActual)==tiempoMuestreo){
    velDerecha=conteoDePulsosDerecha*correccionSegundosDeTiempoMuestreo;
    velIzquierda=conteoDePulsosIzquierda*correcionSegundosDeTiempoMuestreo;
    conteoDePulsosDerecha=0;
    conteoDePulsosIzquierda=0;
    tiempoActual=millis();
  }

}
void ControlProporcionalDerecha(){
  error=velRef-velDerecha;
  vMD=constrain(setPoint+error*Kp,0,255);  
}

void loop() {
  Avanzar();
  Velocidad();
  ControlProporcionalDerecha();
  Serial.print("Velocidad rueda derecha=");
  Serial.print(velDerecha);
  Serial.print(" error =");
  Serial.print(error);
  Serial.print("PWM= ");
  Serial.println(vMD);
}


void ContarPulsosRuedaDerecha(){
  //Esta funcion lleva la cuenta de los pulsos del encoder C1 en la rueda derecha
    conteoDePulsosDerecha++; //Utilicado para el cálculo de velocidad
}

void ContarPulsosRuedaIzquierda(){
  //Esta funcion lleva la cuenta de los pulsos del enconder C1 en la rueda izquierda
    conteoDePulsosIzquierda++; //Utilizado para el cálculo de velocidad 
}


