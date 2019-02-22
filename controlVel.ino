//motor Izquierda conectado a A1N1 y AIN2
//motor Derecha conectado a BIB1 y BIN2
//PWM min para operación es de 27
#define PWMA  10 //Speed control
#define AIN1  11 //Direction
#define AIN2  12
#define PWMB  9
#define BIN1  6
#define BIN2  5

const int tiempoMuestreo=50;
const int conversionSegundosTiempoMuestreo=(60*1000)/tiempoMuestreo;
const int pulsosPorRev=206;
const int limiteSuperiorCicloTrabajo=255;
const int limiteInferiorCicloTrabajo=0;

int contPulsosDerecha=0;
int contPulsosIzquierda=0;
int contDistanciaDer=0;
int contDistanciaIzq=0;
int distLinealRuedaDerecha=0;
int distLinealRuedaIzquierda=0;
int desplazamientoLineal=0;
int vMD=25;//valor bin proporcional al duty cicle motor derecho
int vMI=25;// valor bin proporcional al duty cicle motor izquierdo
int tiempoActual=0;
int velActualDerecha=0;  //dato en RPM
int velActualIzquierda=0; //dato en RPM
const int tiempoGiro=0;//Unidad multiplicada por la frecuencia de muestreo. Teóricamente debería ser 1.5s
const int velRequerida=30; //unidades RPM
const int velRequeridaGiro=30;
const int gradosGiro=90;
const float KpVelocidad=1; //constante control proporcional
const float KdVelocidad=0.5; //constante control derivativo
const float KiVelocidad=0.1; //constante control integral
const float KpGiro=1; //constante control proporcional
const float KdGiro=0.5; //constante control derivativo
const float KiGiro=0.1; //constante control integral
const int errorMinIntegral=-3000;
const int errorMaxIntegral=3000;
const float milimetrosPorPulso=0.6777184466; //avance lineal 139.5mm/rev dividido por 206 pulsos.
bool giro=false;
bool avance=false;
int contTemp=0;
int state=0;


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


void loop() {
  if((millis()-tiempoActual)>=tiempoMuestreo){
    tiempoActual=millis();
    Velocidad();
    Distancia();
    Avanzar();
    Giro();
    Parar();
    contTemp++; //depende del tiempo de muestreo  contTemp*tiempoMuestreo=> ms por evento    
    if(contTemp>=TiempoGiro(gradosGiro)){
        contTemp=0; 
        if(state==0){
          state=1;
          giro=false;
          avance=true;
        }
        else if(state==1){
          avance=false;
          giro=false;
          state=2;
        }
        else if(state==2){
          avance=false;
          giro=true;
          state=3;
        }
        else if(state==3){
          state=0;
          giro=false;
          avance=false;
        }    
    }
   
  }  
}

void Giro(){
  //la función correspondiente realiza el giro a la derecha
  if(giro==true){
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
    vMI=ControlVelocidadGiroIzquierda(vMI,velRequeridaGiro,velActualIzquierda);
    analogWrite(PWMA, vMI);
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW); 
    vMD=ControlVelocidadGiroDerecha(vMD,velRequeridaGiro,velActualDerecha);
    analogWrite(PWMB, vMD);
  }
}
int TiempoGiro(int grados){
  //t=(n vueltas*60s/min)/(rpm)*1/tiempoMuestreo
  //La siguiente fórmula calcula la cantidad de eventos que contTemp debe llevar a cabo para dar n vueltas
  //Función que se requiere para dar el giro en los grados 
  // grados/180 representa la cantidad de revoluciones que se debe dar para plantear el giro 
  int contGiro=(grados*60*1000)/(velRequeridaGiro*tiempoMuestreo*180);
  return  contGiro;
}

int ControlVelocidadDerecha(int command, int velRef, int velActual){
  //Funcion para implementar el control PID de la velocidad de la rueda Derecha
  float pidTermDer=0;
  int errorDer=0;
  static int errorAnteriorDer=0;
  static int sumErrorDer=0;
  errorDer=abs(velRef)-abs(velActual);
  pidTermDer= (KpVelocidad*errorDer)+KiVelocidad*sumErrorDer+(KdVelocidad*(errorDer-errorAnteriorDer));
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
  pidTermIzq= (KpVelocidad*errorIzq)+KiVelocidad*sumErrorIzq+(KdVelocidad*(errorIzq-errorAnteriorIzq));
  sumErrorIzq+= errorIzq;
  sumErrorIzq=constrain(sumErrorIzq,errorMinIntegral,errorMaxIntegral);
  errorAnteriorIzq=errorIzq;
  return constrain(command + int (pidTermIzq),limiteInferiorCicloTrabajo,limiteSuperiorCicloTrabajo);  
}
int ControlVelocidadGiroDerecha(int command, int velRef, int velActual){
  //Funcion para implementar el control PID de la velocidad de la rueda Derecha para el giro
  float pidTermGiroDer=0;
  int errorGiroDer=0;
  static int errorAnteriorGiroDer=0;
  static int sumErrorGiroDer=0;
  errorGiroDer=abs(velRef)-abs(velActual);
  pidTermGiroDer= (KpGiro*errorGiroDer)+KiGiro*sumErrorGiroDer+(KdGiro*(errorGiroDer-errorAnteriorGiroDer));
  sumErrorGiroDer+= errorGiroDer;
  sumErrorGiroDer=constrain(sumErrorGiroDer,errorMinIntegral,errorMaxIntegral);
  errorAnteriorGiroDer=errorGiroDer;
  return constrain(command + int (pidTermGiroDer),limiteInferiorCicloTrabajo,limiteSuperiorCicloTrabajo);  
}
int ControlVelocidadGiroIzquierda(int command, int velRef, int velActual){
  //Funcion que permite implementar el control PID de la rueda izquierda para el giro
  float pidTermGiroIzq=0;
  int errorGiroIzq=0;
  static int errorAnteriorGiroIzq=0;
  static int sumErrorGiroIzq=0;
  errorGiroIzq=abs(velRef)-abs(velActual);
  pidTermGiroIzq= (KpGiro*errorGiroIzq)+KiGiro*sumErrorGiroIzq+(KdGiro*(errorGiroIzq-errorAnteriorGiroIzq));
  sumErrorGiroIzq+= errorGiroIzq;
  sumErrorGiroIzq=constrain(sumErrorGiroIzq,errorMinIntegral,errorMaxIntegral);
  errorAnteriorGiroIzq=errorGiroIzq;
  return constrain(command + int (pidTermGiroIzq),limiteInferiorCicloTrabajo,limiteSuperiorCicloTrabajo);  
}




void Avanzar(){
  //Permite el avance del carro al poner las patillas correspondientes del puente H en alto
  if(avance==true){
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, HIGH);
  vMI=ControlVelocidadIzquierda(vMI,velRequerida,velActualIzquierda);
  analogWrite(PWMA, vMI);
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, HIGH); 
  vMD=ControlVelocidadDerecha(vMD,velRequerida,velActualDerecha);
  analogWrite(PWMB, vMD); 
  }
}

void Parar(){
  //Función que permite detener el carro.
  if(avance==false && giro==false){
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, LOW);
    analogWrite(PWMA, vMI);
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, LOW); 
    analogWrite(PWMB, vMD);
  }
}
  


void Velocidad(){
  //Esta función calcula las rpm en cada una de las ruedas 
    velActualDerecha=contPulsosDerecha*conversionSegundosTiempoMuestreo/pulsosPorRev; //conversión de pulsos/s a RPM
    velActualIzquierda=contPulsosIzquierda*conversionSegundosTiempoMuestreo/pulsosPorRev;
    contPulsosDerecha=0;
    contPulsosIzquierda=0;
}
void Distancia(){
  //se realiza el cálculo de la distancia lineal recorrida por cada una de las ruedas y luego se realiza el promedio, utiliza el modelo de odometría
  distLinealRuedaDerecha=contDistanciaDer*milimetrosPorPulso;
  distLinealRuedaIzquierda=contDistanciaIzq*milimetrosPorPulso;
  desplazamientoLineal=(distLinealRuedaDerecha+distLinealRuedaIzquierda)/2;
}

void PulsosRuedaDerecha(){
  //Esta funcion lleva la cuenta de los pulsos del encoder C1 en la rueda derecha
    contPulsosDerecha++; //Utilicado para el cálculo de velocidad
    contDistanciaDer++;
}

void PulsosRuedaIzquierda(){
  //Esta funcion lleva la cuenta de los pulsos del enconder C1 en la rueda izquierda
    contPulsosIzquierda++; //Utilizado para el cálculo de velocidad 
    contDistanciaIzq++;
}

