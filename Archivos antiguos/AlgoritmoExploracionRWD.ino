//motor Izquierda conectado a A1N1 y AIN2
//motor Derecha conectado a BIB1 y BIN2
//PWM min para operación es de 27
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
const float avanceLineal=139.5;//139.5mm/rev
const float conversionSegundosTiempoMuestreo=12000;
const float pulsosPorMilimetro=(factorEncoder*pulsosPorRev)/avanceLineal; //avance lineal 139.5mm/rev dividido por 206*2 pulsos.
const int velRequerida=40; //unidades RPM
const int distRequerida=200; //unidades mm 

//constantes para control de mov.
const float KpVelocidad=0.5; //constante control proporcional
const float KdVelocidad=0.1; //constante control derivativo
const float KiVelocidad=0.01; //constante control integral
const float KpParo=0.4; //constante control proporcional
const float KdParo=0.1;//constante control derivativo
const float KiParo=0.015; //constante control integral

//constantes para control de giro
const float KpGiro=20; //constante control proporcional
const float KdGiro=2;//constante control derivativo
const float KiGiro=0.08; //constante control integral
const int errorMinIntegral=-100;
const int errorMaxIntegral=100;
const int limiteSuperiorCicloTrabajoGiro=130;
const int limiteInferiorCicloTrabajoGiro=-130;
const float radioGiro=65.0;// estaba a 65.75
const float errorGiroMax=0.5;
const float errorGiroPermitido=errorGiroMax/2;

//Variables para mov. lineal
const float conversionVelocidad=(conversionSegundosTiempoMuestreo/(pulsosPorRev*factorEncoder));
int contDistanciaDer=0;
int contDistanciaIzq=0;
int velActualDerecha=0;  //dato en RPM
int velActualIzquierda=0; //dato en RPM
int contPulsosDerecha=0;
int contPulsosIzquierda=0;
int vMD=25;//valor bin proporcional al duty cicle motor derecho
int vMI=25;// valor bin proporcional al duty cicle motor izquierdo
long tiempoActual=0;
float distLinealRuedaDerecha=0;
float distLinealRuedaIzquierda=0;
float desplazamientoLineal=0;
float pidTermDer=0;
float errorDer=0;
float errorAnteriorDer=0;
float sumErrorDer=0;
float pidTermIzq=0;
float errorIzq=0;
float errorAnteriorIzq=0;
float sumErrorIzq=0;

//Variables para el giro
float contGiroDer=0;
float contGiroIzq=0;
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
int contGiroCorrecto=0; //Contador que me indica si ya el robot está en la posición correcta por N cantidad de veces.

//Variables para máquina de estados
bool giro=false;
bool avance=true;
bool pare=false;

//Variables para interrupción encoders
boolean A,B,C,D;
byte state,state2;
byte statep,statep2;
int randNumber=1;

//Variables para el RandomWalk


bool giroAnteriorIzquierda=0;
bool giroAnteriorDerecha=0;
int direccionDeseada=1;//random(1,5);//asigna direccion N,S,E,O 
int direccionActual=1; // 1=N, 2=E, 3=S, 4=O
bool giroTerminado=1;
bool detectaObjeto=0;
int anguloGiro = 90;
bool direccionAdelanteDisponible=1;
bool direccionIzquierdaDisponible=1;
bool direccionDerechaDisponible=1;
int opcionGiro=0;// puede tener 3 valores, 1=Izquierda 2=Adelante, 3=Derecha
bool asignarDireccionDeseada=0;


void setup() {
  // put your setup code here, to run once:
  pinMode(PWMA, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(A4, INPUT_PULLUP);
  attachInterrupt(A0, PulsosRuedaDerechaC1,CHANGE);  //conectado el contador C1 rueda derecha
  attachInterrupt(A1, PulsosRuedaDerechaC2,CHANGE); //conectado el contador C1 rueda izquierda
  attachInterrupt(A2, PulsosRuedaIzquierdaC1,CHANGE);
  attachInterrupt(A3, PulsosRuedaIzquierdaC2,CHANGE);
  attachInterrupt(digitalPinToInterrupt(A4), DeteccionObjeto,FALLING);
  Serial.begin(9600);
  tiempoActual=micros();
  randomSeed(analogRead(A5));
  delay(3000);
  //direccionDeseada=random(1,5);//asigna direccion N,S,E,O

}


void loop() {
   
   
  if((micros()-tiempoActual)>=tiempoMuestreo){
    tiempoActual=micros();
    
    if(avance==true){
        Distancia();
        ReinicioEstadosDisponiblesGiro();
        
        if(desplazamientoLineal>=distRequerida){
          desplazamientoLineal=0;
          avance=false;
          pare=true;
        }
        Velocidad();
        Avanzar();
      }
      
      if(giro==true){
        if(giroTerminado ){
        EscogerDireccion();
        
        }
        Giro(anguloGiro);
      }
  
      if(pare==true){
        Parar();
        giro=true;
        contGiroDer=0;
        contGiroIzq=0;
      }
      
    }
}



void AsignarDireccionDeseada(int direccion){//direccion puede valer 1(Izquierda) o 3(Derecha)
  //Esta funcion se utiliza únicamente si hubo un obstáculo en dirección deseada, reasigna la dirección deseada con base en la actualy el giro realizado
   direccionDeseada=direccionActual;   
}
void ReinicioEstadosDisponiblesGiro(){
  direccionAdelanteDisponible=1;
  direccionDerechaDisponible=1;
  direccionIzquierdaDisponible=1;
}

void AsignaDireccionSinObstaculo(){
  if(direccionActual==direccionDeseada){
          opcionGiro=random(1,4);// 1=izquierda 2=avanza 3=derecha
  }
  else if(direccionActual!=direccionDeseada){
    if(abs(direccionActual-direccionDeseada)==2){//completamente en la direccion opuesta
      opcionGiro=4;
    }
    else if(giroAnteriorIzquierda==1){
      opcionGiro=random(2,4);//gire derecha o adelante
    }
    else if(giroAnteriorDerecha==1){
      opcionGiro=random(1,3);//gire izquierda o adelante
    }
    else if(giroAnteriorDerecha==0 && giroAnteriorIzquierda==0){
      //Asume que siempre se coloca en direccion Norte
      if(direccionDeseada==2){//si la deseada es  Este
        opcionGiro=3; //Gire a la izquierda
      }
      if(direccionDeseada==4){//si la deseada es Oeste
        opcionGiro=1;//gire a la derecha
      }
    }
  }


}
void AsignaDireccionConObstaculo(){
  if(direccionActual==direccionDeseada){
      direccionAdelanteDisponible=0;
      asignarDireccionDeseada=1;
      if(direccionDerechaDisponible==1 && direccionIzquierdaDisponible==1){
        opcionGiro=random(1,3);//si es 1, gire izquierda
        if(opcionGiro==1){
        }
        if(opcionGiro==2){//sino, gire derecha
          opcionGiro=3;
        }
      }
      else if(direccionDerechaDisponible==0 && direccionIzquierdaDisponible==1){
        opcionGiro=1;
      }
      else if(direccionDerechaDisponible==1 && direccionIzquierdaDisponible==0){
        opcionGiro=3;
      }
      else if(direccionDerechaDisponible==0 && direccionIzquierdaDisponible==0 && direccionAdelanteDisponible==0){
        opcionGiro=4;
      }
             
  }
  else if( direccionActual!=direccionDeseada){
    if(direccionAdelanteDisponible==1){
    switch(direccionDeseada){
        case 1:
          if(direccionActual==4){
             opcionGiro=3;
          }
          if(direccionActual==2){
            opcionGiro=1;
          }
          break;
        case 4:
          if(direccionActual==1){
            opcionGiro=1;
          }
          if(direccionActual==3){
            opcionGiro=3;
          }
          break;
         case 2:
          if(direccionActual==1){
            opcionGiro=3;
          }
          if(direccionActual==3){
            opcionGiro=1;
          }
          break;
          case 3:
          if(direccionActual==2){
            opcionGiro=3;
           }
          if(direccionActual==4){
            opcionGiro=1;
           }
          break;
         default:
          break;
      }
      if(opcionGiro==1){
        direccionDerechaDisponible=0;
      }
      if(opcionGiro==3){
        direccionIzquierdaDisponible=0;
      }
    
  }
  else if(direccionAdelanteDisponible==0 && (direccionDerechaDisponible==1 || direccionIzquierdaDisponible==1)){
    opcionGiro=4;
    if(giroAnteriorDerecha==1){direccionIzquierdaDisponible=0;}
    if(giroAnteriorIzquierda==1){direccionDerechaDisponible=0;}
  }
  else{
    Serial.print("DENTRO");
    switch(direccionDeseada){
        case 1:
          if(direccionActual==4){
             opcionGiro=1;
          }
          if(direccionActual==2){
            opcionGiro=3;
          }
          break;
        case 4:
          if(direccionActual==1){
            opcionGiro=3;
          }
          if(direccionActual==3){
            opcionGiro=1;
          }
          break;
         case 2:
          if(direccionActual==1){
            opcionGiro=1;
          }
          if(direccionActual==3){
            opcionGiro=3;
          }
          break;
          case 3:
          if(direccionActual==2){
            opcionGiro=1;
           }
          if(direccionActual==4){
            opcionGiro=3;
           }
          break;
         default:
          break;
      }
  }
 }  
}



void EscogerDireccion(){

    if(digitalRead(A4)==0){//hay un obstaculo frente
      //Si hay obstaculo opcionGiro se limita a 1=Izquierda,3=Derecha,5=Giro180
      //Utiliza las memorias de direccion Disponible, las cuales al avanzar regresan a 1.
      AsignaDireccionConObstaculo();
     }
    else if(digitalRead(A4)==1){//Si no hay obstaculo
      AsignaDireccionSinObstaculo();
    }
    switch (opcionGiro){
      case 1://izquierda
        giroAnteriorIzquierda=1;
        giroAnteriorDerecha=0;
        if(direccionActual==1) {direccionActual=4;}
        else {direccionActual--;}
        anguloGiro=90;
        break;
    
      case 2: //Adelante
        direccionActual=direccionActual;
        anguloGiro=0;
        break;

      case 3://derecha
        giroAnteriorDerecha=1;
        giroAnteriorIzquierda=0;
        if(direccionActual==4){direccionActual=1;}
        else {direccionActual++;}
        anguloGiro=-90;
        break;
       case 4:
        anguloGiro=180;
        direccionActual=direccionDeseada+2;
        if(direccionActual==5){direccionActual=1;}
        if(direccionActual==6){direccionActual=2;}
        direccionDeseada=direccionActual;
        giroAnteriorDerecha=0;
        giroAnteriorIzquierda=0;
        break;   
      default:
        break;
    }
 
      if(direccionActual==1){
    Serial.print("^");
   }
    if(direccionActual==2){
    Serial.print(">");
   }
    if(direccionActual==3){
    Serial.print("v");
   }
    if(direccionActual==4){
    Serial.print("<");
   }
   Serial.print(" ");
      if(direccionDeseada==1){
    Serial.print("^");
   }
   if(direccionDeseada==2){
    Serial.print(">");
   }
    if(direccionDeseada==3){
    Serial.print("v");
   }
    if(direccionDeseada==4){
    Serial.print("<");
   }
    Serial.print("   ");
    if(direccionIzquierdaDisponible==1){
      Serial.print("<");
    }
   
    if(direccionIzquierdaDisponible==0){Serial.print("*");}
    if(direccionAdelanteDisponible==1){
      Serial.print("^");
    }
    if(direccionAdelanteDisponible==0){Serial.print("*");}
    if(direccionDerechaDisponible==1){
      Serial.println(">");
    }
     if(direccionDerechaDisponible==0){Serial.println("*");}
     giroTerminado=0;
}

void DeteccionObjeto(){
  if(detectaObjeto==0 && giroTerminado==1){
   detectaObjeto=1;
   Serial.println("OBJETO!!");
   digitalWrite(13,HIGH);
   pare=true;
   avance=false;
   giro=false;
  } 
   
}

void Parar(){
  if(giro==0 && avance==0 && pare==true){
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, LOW);
    analogWrite(PWMA, vMI);
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, LOW); 
    analogWrite(PWMB, vMD);
    delay(250);
    pare=false;
  }
  contDistanciaDer=0;
  contDistanciaIzq=0;
}
void Avanzar(){
  //Permite el avance del carro al poner las patillas correspondientes del puente H en alto
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, HIGH);
  vMI=ControlVelocidadIzquierda(vMI,velRequerida,velActualIzquierda);
  analogWrite(PWMA, vMI);
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, HIGH); 
  vMD=ControlVelocidadDerecha(vMD,velRequerida,velActualDerecha);
  analogWrite(PWMB, vMD); 
}
void Giro(int grados){
//la función correspondiente realiza el giro a la derecha
  posActualDerecha=ConvDespAngular(contGiroDer);
  vMD=ControlPosGiroRuedaDerecha(grados,posActualDerecha);
  posActualIzquierda=ConvDespAngular(contGiroIzq);
  vMI=ControlPosGiroRuedaIzquierda(grados,posActualIzquierda);

  if(vMD==0 && vMI==0){
    //digitalWrite(13,HIGH);
    contGiroCorrecto++;
    if(contGiroCorrecto>20){ //Si ya llevo 10 tiempos de muestreo en la pos. correcta, termina el giro.
      giro=false;
      pare=true;
      Parar();
      if(digitalRead(A4)==1){
      detectaObjeto=0;
      if(asignarDireccionDeseada==1){
        AsignarDireccionDeseada(opcionGiro);
        asignarDireccionDeseada=0;
      }
      avance=true;
      giro=false;
      pare=false;
      }
      else if(digitalRead(A4)==0){
        giro=true;
        pare=false;
        avance=false;
        contGiroDer=0;
        contGiroIzq=0;
         
      }
      contDistanciaDer=0;
      contDistanciaIzq=0;
      desplazamientoLineal=0;
      contGiroCorrecto = 0;
      giroTerminado=1;
      digitalWrite(13,LOW);
    }
  }
  else{
   // digitalWrite(13,LOW);
  }
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
    pidTermGiroDer=map(pidTermGiroDer,0,55,45,limiteSuperiorCicloTrabajoGiro); //0,55,35
  }
  if(pidTermGiroDer<0){
    pidTermGiroDer=map(pidTermGiroDer,-55,0,limiteInferiorCicloTrabajoGiro,-45); //-55,0,-35
  }
  return constrain( int (pidTermGiroDer),limiteInferiorCicloTrabajoGiro,limiteSuperiorCicloTrabajoGiro);  
}

int ControlPosGiroRuedaIzquierda( float posRef, float posActual){
  //Funcion para implementar el control PID de la velocidad de la rueda Derecha para el giro
  errorGiroIzq=posRef-posActual;
  sumErrorGiroIzq+= errorGiroIzq;
  sumErrorGiroIzq=constrain(sumErrorGiroIzq,errorMinIntegral,errorMaxIntegral);
  pidTermGiroIzq= (KpGiro*errorGiroIzq)+KiGiro*sumErrorGiroIzq+(KdGiro*(errorGiroIzq-errorAnteriorGiroIzq));
  errorAnteriorGiroIzq=errorGiroIzq;
  if(errorGiroIzq>-errorGiroPermitido && errorGiroIzq<errorGiroPermitido){
    pidTermGiroIzq=0;
  }
   if(pidTermGiroIzq>0){
    pidTermGiroIzq=map(pidTermGiroIzq,0,55,45,limiteSuperiorCicloTrabajoGiro); //0,55,35
  }
  if(pidTermGiroIzq<0){
    pidTermGiroIzq=map(pidTermGiroIzq,-55,0,limiteInferiorCicloTrabajoGiro,-45); //-55,0,-35
  }
  return constrain( int (pidTermGiroIzq),limiteInferiorCicloTrabajoGiro,limiteSuperiorCicloTrabajoGiro);  
}

float ConvDespAngular(float pulsos){
  despAngular=(pulsos*avanceLineal*180)/(radioGiro*factorEncoder*pulsosPorRev*3.141592);
  return despAngular;
}

void Velocidad(){
  //Esta función calcula las rpm en cada una de las ruedas 
    velActualDerecha=contPulsosDerecha*conversionVelocidad; //conversión de pulsos/s a RPM
    velActualIzquierda=contPulsosIzquierda*conversionVelocidad;
    contPulsosDerecha=0;
    contPulsosIzquierda=0;
}

void Distancia(){
  //Realiza el cálculo de la distancia lineal recorrida por cada una de las ruedas y luego se realiza el promedio, utiliza el modelo de odometría
  distLinealRuedaDerecha=abs(float(contDistanciaDer)*(1/pulsosPorMilimetro));
  distLinealRuedaIzquierda=abs(float(contDistanciaIzq)*(1/pulsosPorMilimetro));
  desplazamientoLineal+=(distLinealRuedaDerecha+distLinealRuedaIzquierda)/2.0;
  contDistanciaIzq=0;
  contDistanciaDer=0;
}



int ControlVelocidadDerecha(int command, int velRef, int velActual){
  //Funcion para implementar el control PID de la velocidad de la rueda Derecha
  errorDer=abs(velRef)-abs(velActual);
  sumErrorDer+= errorDer;
  sumErrorDer=constrain(sumErrorDer,errorMinIntegral,errorMaxIntegral);
  pidTermDer= (KpVelocidad*errorDer)+KiVelocidad*sumErrorDer+(KdVelocidad*(errorDer-errorAnteriorDer));
  errorAnteriorDer=errorDer;
  return constrain(command + int (pidTermDer),limiteInferiorCicloTrabajo,limiteSuperiorCicloTrabajo);  
}

int ControlVelocidadIzquierda(int command, int velRef, int velActual){
  //Funcion que permite implementar el control PID de la rueda izquierda
  errorIzq=abs(velRef)-abs(velActual);
  sumErrorIzq+= errorIzq;
  sumErrorIzq=constrain(sumErrorIzq,errorMinIntegral,errorMaxIntegral);
  errorAnteriorIzq=errorIzq;
  pidTermIzq= (KpVelocidad*errorIzq)+KiVelocidad*sumErrorIzq+(KdVelocidad*(errorIzq-errorAnteriorIzq));
  return constrain(command + int (pidTermIzq),limiteInferiorCicloTrabajo,limiteSuperiorCicloTrabajo);  
}



//Interrupciones:
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
      if (statep == 2){
        contGiroDer--;
        contPulsosDerecha--;
        contDistanciaDer--;
      }
      if (statep == 4){
        contGiroDer++;
        contPulsosDerecha++;
        contDistanciaDer++;
      }
      break;
    }
    case 2:
    {
      if (statep == 1){
        contGiroDer++;
        contPulsosDerecha++;
        contDistanciaDer++;
      }
      if (statep == 3){
        contGiroDer--;
        contPulsosDerecha--;
        contDistanciaDer--;
      }
      break;
    }
    case 3:
    {
      if (statep == 2){
        contGiroDer++;
        contPulsosDerecha++;
        contDistanciaDer++;
      }
      if (statep == 4){
        contGiroDer--;
        contPulsosDerecha--;
        contDistanciaDer--;
      }
      break;
    }
    default:
    {
      if (statep == 1){
        contGiroDer--;
        contPulsosDerecha--;
        contDistanciaDer--;
      }
      if (statep == 3){
        contGiroDer++;
        contPulsosDerecha++;
        contDistanciaDer++;
      }
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
      if (statep2 == 2){ 
        contGiroIzq--;
        contPulsosIzquierda--;
        contDistanciaIzq--;
      }
      if (statep2 == 4){ 
        contGiroIzq++;
        contPulsosIzquierda++;
        contDistanciaIzq++;
      }
      break;
    }
    case 2:
    {
      if (statep2 == 1){ 
        contGiroIzq++;
        contPulsosIzquierda++;
        contDistanciaIzq++;
      }
      if (statep2 == 3){ 
        contGiroIzq--;
        contPulsosIzquierda--;
        contDistanciaIzq--;
      }
      break;
    }
    case 3:
    {
      if (statep2 == 2){ 
        contGiroIzq++;
        contPulsosIzquierda++;
        contDistanciaIzq++;
      }
      if (statep2 == 4){ 
        contGiroIzq--;
        contPulsosIzquierda--;
        contDistanciaIzq--;
      }
      break;
    }
    default:
    {
      if (statep2 == 1){ 
        contGiroIzq--;
        contPulsosIzquierda--;
        contDistanciaIzq--;
      }
      if (statep2 == 3){ 
        contGiroIzq++;
        contPulsosIzquierda++;
        contDistanciaIzq++;
      }
    }
  }
  statep2 = state2;
}


