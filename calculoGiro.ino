//motor A connected between A01 and A02, left wheel
//motor B connected between B01 and B02, right wheel
//PWM min para operación es de 27
int PWMA = 10; //Speed control
int AIN1 = 11; //Direction
int AIN2 = 12; //Direction
int PWMB = 9 ;
int BIN1 = 6;
int BIN2 = 5;
int conteoDePulsosDerecha = 0;
int conteoDePulsosIzquierda = 0;
int vMD = 100;//valor bin equivalente al duty cicle motor derecho
int vMI = 100;// valor bin equivalente al duty cicle motor izquierdo
int tiempoActual = 0;
int velDerecha = 0;
int velIzquierda = 0;
const int velRef = 500; //pulsos por segundo
int error = 0;
int setPoint = 100;
int Kp = 1;
const int tiempoMuestreo = 100; //medido en milisegundos
const int correccionSegundosDeTiempoMuestreo = 10;
const int limiteSuperiorCicloTrabajo = 255;
const int limiteInferiorCicloTrabajo = 0;
const int pausaAvanceGiro = 500;
const int pausaDentroDeGiro = 10;
int conteoVelocidadActualDerecha = 0;

void setup() {
  pinMode(PWMA, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  attachInterrupt(A0, ContarPulsosRuedaDerecha, RISING);
  attachInterrupt(A2, ContarPulsosRuedaIzquierda, RISING);
  Serial.begin(9600);
  tiempoActual = millis();
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

void Girar90(){
  //La función hacer girar 90° al robot.

  //Detengo por completo el móvil
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, LOW);
  delay(pausaAvanceGiro); //Pausa de 500 milis
  //Reseteo los contadores
  conteoDePulsosDerecha = 0;
  conteoDePulsosIzquierda = 0;
  //Pongo en marcha el giro
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, HIGH);
  analogWrite(PWMA, 60);
  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, LOW);
  analogWrite(PWMB, 60);

  //Esperar en el while hasta que los contadores hayan contado lo respectivo a un giro de 90°
  while(conteoDePulsosIzquierda <= 103){
    delay(pausaDentroDeGiro); //Pausa de 10 milis
  }

  //Detengo por completo el móvil 
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, LOW);
  delay(pausaAvanceGiro); //Pausa de 500 milis
  //Reseteo los contadores
  conteoDePulsosDerecha = 0;
  conteoDePulsosIzquierda = 0;
  //Pongo en marcha el avance
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, HIGH);
  analogWrite(PWMA, vMI);
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, HIGH);
  analogWrite(PWMB, vMD);
}

void loop() {
  Avanzar();
  delay(pausaAvanceGiro);
  Girar90();
}

void ContarPulsosRuedaDerecha(){
  //Esta funcion lleva la cuenta de los pulsos del encoder C1 en la rueda derecha
  conteoDePulsosDerecha++; //Utilizado para el cálculo de velocidad
}

void ContarPulsosRuedaIzquierda(){
  //Esta funcion lleva la cuenta de los pulsos del enconder C1 en la rueda izquierda
  conteoDePulsosIzquierda++; //Utilizado para el cálculo de velocidad 
}
