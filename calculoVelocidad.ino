//motor A connected between A01 and A02
//motor B connected between B01 and B02
//PWM min para operación es de 27
int PWMA = 10; //Speed control
int AIN1 = 11; //Direction
int AIN2 = 12; //Direction
int PWMB=9 ;
int BIN1=6;
int BIN2=5;
int cDC1=0;
bool cDC2=1;//motor invertido respecto al otro
int cIC1=0;
bool cIC2=0;
int vMD=100;//valor bin equivalente al duty cicle motor derecho
int vMI=100;// valor bin equivalente al duty cicle motor izquierdo
int tiempoActual=0;
int velDerecha=0;
int velIzquierda=0;
const int velRef=500; //pulsos por segundo
int error=0;
int setPoint=100;
int Kp=1;
const int tiempoMuestreo=500;
const int limiteSuperiorCicloTrabajo=255;
const int limiteInferiorCicloTrabajo=0;

void setup() {
  // put your setup code here, to run once:
  pinMode(PWMA, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  attachInterrupt(A0, DC1, RISING);
  attachInterrupt(A1, DC2, CHANGE);
  attachInterrupt(A2, IC1, RISING);
  attachInterrupt(A3, IC2, CHANGE);
  Serial.begin(9600);
  tiempoActual=millis();
}


void Avanzar(){
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, HIGH);
  analogWrite(PWMA, vMI);
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, HIGH);
  analogWrite(PWMB, vMD);
  
}
void Velocidad(){
  
  if((millis()-tiempoActual)==tiempoMuestreo){
    velDerecha=cDC1*2;
    velIzquierda=cIC1*2;
    cDC1=0;
    cIC1=0;
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


void DC1(){
  if(cDC2==1){
    cDC1++; 
  }
  else{
    cDC1--;
  }
}
void DC2(){
cDC2=!cDC2;
}
void IC1(){
  if(cIC2==1){
    cIC1++;
  }
  else{
    cIC1--;
  }
}
void IC2(){
cIC2=!cIC2;
}

