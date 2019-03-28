///motor Izquierda conectado a A1N1 y AIN2
//motor Derecha conectado a BIB1 y BIN2
//PWM min para operación es de 27
#define PWMA  10 //Speed control
#define AIN1  11 //Direction
#define AIN2  12
#define PWMB  9
#define BIN1  6
#define BIN2  5

const int tiempoMuestreo=1;
const int conversionSegundosTiempoMuestreo=(60*1000)/tiempoMuestreo;
const int pulsosPorRev=206;
const int limiteSuperiorCicloTrabajo=255;
const int limiteInferiorCicloTrabajo=0;

int contPulsosDerecha=0;
int contPulsosIzquierda=0;
int contDistanciaDer=0;
int contDistanciaIzq=0;
float distLinealRuedaDerecha=0;
float distLinealRuedaIzquierda=0;
float desplazamientoLineal=0;
int vMD=25;//valor bin proporcional al duty cicle motor derecho
int vMI=25;// valor bin proporcional al duty cicle motor izquierdo
int tiempoActual=0;
int velActualDerecha=0;  //dato en RPM
int velActualIzquierda=0; //dato en RPM
const int tiempoGiro=0;//Unidad multiplicada por la frecuencia de muestreo. Teóricamente debería ser 1.5s
const int velRequerida=50; //unidades RPM
const int velRequeridaGiro=30;
const int gradosGiro=90;
const float KpVelocidad=0.5; //constante control proporcional
const float KdVelocidad=0.1; //constante control derivativo
const float KiVelocidad=0.01; //constante control integral
const float KpGiroHorario=0.4; //constante control proporcional
const float KdGiroHorario=0.1;//constante control derivativo
const float KiGiroHorario=0.015; //constante control integral
const float KpGiroAntihorario=KpGiroHorario; //constante control proporcional
const float KdGiroAntihorario=KdGiroHorario; //constante control derivativo
const float KiGiroAntihorario=KiGiroHorario; //constante control integral
const float KpParo=0.4; //constante control proporcional
const float KdParo=0.1;//constante control derivativo
const float KiParo=0.015; //constante control integral
const int errorMinIntegral=-3000;
const int errorMaxIntegral=3000;
const float radioGiro=65.75;
const float milimetrosPorPulso=0.6777184466; //avance lineal 139.5mm/rev dividido por 206 pulsos.
bool giro=false;
bool avance=false;
int contTemp=0;
int state=0;
int cTiempoGiro=0;



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
    Serial.print(contDistanciaIzq);
    Serial.print(" ");
    Serial.println(contDistanciaDer);
    tiempoActual=millis();
    Velocidad();
    Distancia();
    Avanzar();
    contTemp++; //depende del tiempo de muestreo  contTemp*tiempoMuestreo=> ms por evento  
    cTiempoGiro++;   
    
    
    if(contTemp>=20000){
        contTemp=0;} 
    if( contTemp>=0 && contTemp<5000){ //giri izq
      giro=true;
      Giro(90);
      avance=false;
    }
    else if( contTemp>=5000 && contTemp<10000){// parar
      if(contTemp==9999){
      contDistanciaDer=0;
      contDistanciaIzq=0;
      }
      cTiempoGiro=0; //prueba
      avance=false;
      giro=false;
      if(contTemp>8000){
        Parar();
        avance=false;
      }
     
    }
    
    else if( contTemp>=10000 && contTemp<15000){ //Girar
      avance=false;
      giro=true;
      Giro(-90);
      
    }
    else if( contTemp>=15000 && contTemp<20000){ // Parar
      if(contTemp==19999){
      contDistanciaDer=0;
      contDistanciaIzq=0;}
      cTiempoGiro=0; //prueba
      giro=false;
      avance=false;
      if(contTemp>18000){
        Parar();
        avance=false;
      }
     
    }       
  }  
}

void Giro(int grados){
  //la función correspondiente realiza el giro a la derecha
  if(giro==true && (cTiempoGiro<=TiempoGiro(abs(grados))) && grados>0){
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
    vMI=ControlVelocidadGiroRuedaIzquierdaAntihorario(vMI,velRequeridaGiro,velActualIzquierda);
    analogWrite(PWMA, vMI);
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH); 
    vMD=ControlVelocidadGiroRuedaDerechaAntihorario(vMD,velRequeridaGiro,velActualDerecha);
    analogWrite(PWMB, vMD);
   }
   
  if(giro==true && (cTiempoGiro<=TiempoGiro(abs(grados))) && grados<0){
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
    vMI=ControlVelocidadGiroRuedaIzquierdaHorario(vMI,velRequeridaGiro,velActualIzquierda);
    analogWrite(PWMA, vMI);
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW); 
    vMD=ControlVelocidadGiroRuedaDerechaHorario(vMD,velRequeridaGiro,velActualDerecha);
    analogWrite(PWMB, vMD); 
  }

  if(cTiempoGiro>=TiempoGiro(abs(grados))){
    Parar();
  }
}
void Parar(){
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);
  analogWrite(PWMA, vMI);
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, LOW); 
  analogWrite(PWMB, vMD);
  
}

int ControlVelocidadGiroRuedaDerechaHorario(int command, int velRef, int velActual){
  //Funcion para implementar el control PID de la velocidad de la rueda Derecha para el giro
  float pidTermGiroDerHorario=0;
  int errorGiroDerHorario=0;
  static int errorAnteriorGiroDerHorario=0;
  static int sumErrorGiroDerHorario=0;
  errorGiroDerHorario=abs(velRef)-abs(velActual);
  pidTermGiroDerHorario= (KpGiroHorario*errorGiroDerHorario)+KiGiroHorario*sumErrorGiroDerHorario+(KdGiroHorario*(errorGiroDerHorario-errorAnteriorGiroDerHorario));
  sumErrorGiroDerHorario+= errorGiroDerHorario;
  sumErrorGiroDerHorario=constrain(sumErrorGiroDerHorario,errorMinIntegral,errorMaxIntegral);
  return constrain(command + int (pidTermGiroDerHorario),limiteInferiorCicloTrabajo,limiteSuperiorCicloTrabajo);  
}
int ControlVelocidadGiroRuedaIzquierdaHorario(int command, int velRef, int velActual){
  //Funcion que permite implementar el control PID de la rueda izquierda para el giro
  float pidTermGiroIzqHorario=0;
  int errorGiroIzqHorario=0;
  static int errorAnteriorGiroIzqHorario=0;
  static int sumErrorGiroIzqHorario=0;
  errorGiroIzqHorario=abs(velRef)-abs(velActual);
  pidTermGiroIzqHorario= (KpGiroHorario*errorGiroIzqHorario)+KiGiroHorario*sumErrorGiroIzqHorario+(KdGiroHorario*(errorGiroIzqHorario-errorAnteriorGiroIzqHorario));
  sumErrorGiroIzqHorario+= errorGiroIzqHorario;
  sumErrorGiroIzqHorario=constrain(sumErrorGiroIzqHorario,errorMinIntegral,errorMaxIntegral);
  errorAnteriorGiroIzqHorario=errorGiroIzqHorario;
  return constrain(command + int (pidTermGiroIzqHorario),limiteInferiorCicloTrabajo,limiteSuperiorCicloTrabajo);  
}
int ControlVelocidadGiroRuedaDerechaAntihorario(int command, int velRef, int velActual){
  //Funcion para implementar el control PID de la velocidad de la rueda Derecha para el giro
  float pidTermGiroDerAntihorario=0;
  int errorGiroDerAntihorario=0;
  static int errorAnteriorGiroDerAntihorario=0;
  static int sumErrorGiroDerAntihorario=0;
  errorGiroDerAntihorario=abs(velRef)-abs(velActual);
  pidTermGiroDerAntihorario= (KpGiroAntihorario*errorGiroDerAntihorario)+KiGiroAntihorario*sumErrorGiroDerAntihorario+(KdGiroAntihorario*(errorGiroDerAntihorario-errorAnteriorGiroDerAntihorario));
  sumErrorGiroDerAntihorario+= errorGiroDerAntihorario;
  sumErrorGiroDerAntihorario=constrain(sumErrorGiroDerAntihorario,errorMinIntegral,errorMaxIntegral);
  errorAnteriorGiroDerAntihorario=errorGiroDerAntihorario;
  return constrain(command + int (pidTermGiroDerAntihorario),limiteInferiorCicloTrabajo,limiteSuperiorCicloTrabajo);  
}
int ControlVelocidadGiroRuedaIzquierdaAntihorario(int command, int velRef, int velActual){
  //Funcion que permite implementar el control PID de la rueda izquierda para el giro
  float pidTermGiroIzqAntihorario=0;
  int errorGiroIzqAntihorario=0;
  static int errorAnteriorGiroIzqAntihorario=0;
  static int sumErrorGiroIzqAntihorario=0;
  errorGiroIzqAntihorario=abs(velRef)-abs(velActual);
  pidTermGiroIzqAntihorario= (KpGiroAntihorario*errorGiroIzqAntihorario)+KiGiroAntihorario*sumErrorGiroIzqAntihorario+(KdGiroAntihorario*(errorGiroIzqAntihorario-errorAnteriorGiroIzqAntihorario));
  sumErrorGiroIzqAntihorario+= errorGiroIzqAntihorario;
  sumErrorGiroIzqAntihorario=constrain(sumErrorGiroIzqAntihorario,errorMinIntegral,errorMaxIntegral);
  errorAnteriorGiroIzqAntihorario=errorGiroIzqAntihorario;
  return constrain(command + int (pidTermGiroIzqAntihorario),limiteInferiorCicloTrabajo,limiteSuperiorCicloTrabajo);  
}
int TiempoGiro(int grados){
  //t=(n vueltas*60s/min)/(rpm)*1/tiempoMuestreo
  //La siguiente fórmula calcula la cantidad de eventos que contTemp debe llevar a cabo para dar n vueltas
  //el tiempo de muestreo es en ms, se compara con un contador que cambia en ms también
  //Función que se requiere para dar el giro en los grados 
  // la cantidad de revoluciones por giro esta asociada al radio de giro por  rev/206pulsos*pulsos/mmPorPulso*radioGiro*grados(en rad).
  int contGiro=(radioGiro*grados*60*1000*3.14)/(180*pulsosPorRev*milimetrosPorPulso*velRequeridaGiro*tiempoMuestreo);
  return  int(contGiro);
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


  


void Velocidad(){
  //Esta función calcula las rpm en cada una de las ruedas 
    velActualDerecha=contPulsosDerecha*conversionSegundosTiempoMuestreo/pulsosPorRev; //conversión de pulsos/s a RPM
    velActualIzquierda=contPulsosIzquierda*conversionSegundosTiempoMuestreo/pulsosPorRev;
    contPulsosDerecha=0;
    contPulsosIzquierda=0;
    
}
void Distancia(){
  //se realiza el cálculo de la distancia lineal recorrida por cada una de las ruedas y luego se realiza el promedio, utiliza el modelo de odometría
  distLinealRuedaDerecha=float(contDistanciaDer)*milimetrosPorPulso;
  distLinealRuedaIzquierda=float(contDistanciaIzq)*milimetrosPorPulso;
  desplazamientoLineal=(distLinealRuedaDerecha+distLinealRuedaIzquierda)/2.0;
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