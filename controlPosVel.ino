//Código principal para el proyecto PROE
//https://github.com/jcbrenes/PROE

//constantes del robot empleado
const int tiempoMuestreo=10000; //unidades: micro segundos
const float pulsosPorRev=206.0; //cantidad de pulsos de una única salida
const int factorEncoder=4; //cantidad de tipos de pulsos que se están detectando (juego entre las 2 salidas del encoder)
const float circunferenciaRueda=139.5;//Circunferencia de la rueda = 139.5mm 
const float pulsosPorMilimetro=((float)factorEncoder*pulsosPorRev)/circunferenciaRueda; 
const float distanciaCentroARueda=65.0;// Radio de giro del carro, es la distancia en mm entre el centro y una rueda. Estaba a 65.75
const float conversionMicroSaMin=1/(60 * 1000000);// factor de conversion microsegundo (unidades del tiempo muestreo) a minuto
const float conversionMicroSaSDiv=1000000;// factor de conversion microsegundo (unidades del tiempo muestreo) a segundo
const float tiempoMuestreoS= (float)tiempoMuestreo/conversionMicroSaSDiv;

//constantes para control de velocidad
const float velRequerida=250; //unidades mm/s
const float KpVel=2; //constante control proporcional
const float KiVel=20.0 * tiempoMuestreoS; //constante control integral
const float KdVel=0.01 / tiempoMuestreoS ; //constante control derivativo
//constantes para control de giro 
//(ojo que estas constantes están unidas con la constante de tiempo por simplificación de la ecuación)
const float KpGiro=5; //constante control proporcional
const float KiGiro=25.0 * tiempoMuestreoS;//constante control integral
const float KdGiro=0.08 / tiempoMuestreoS; //constante control derivativo

//Constantes para las ecuaciones de control PID
const int errorMinIntegral=-100;
const int errorMaxIntegral=100;
const int limiteSuperiorCicloTrabajoVelocidad=100;
const int limiteInferiorCicloTrabajoVelocidad=0;
const int limiteSuperiorCicloTrabajoGiro=100;
const int limiteInferiorCicloTrabajoGiro=-100;
const int cicloTrabajoMinimo= 45;
const int minCiclosEstacionario= 10;

//Configuración de pines de salida para conexión con el Puente H
const int PWMA = 10; //Control velocidad izquierdo
const int AIN1 = 11; //Dirección 1 rueda izquierda 
const int AIN2 = 12; //Dirección 2 rueda izquierda 
const int PWMB = 9;  //Control velocidad derecha
const int BIN1 = 6;  //Dirección 1 rueda derecha
const int BIN2 = 5;  //Dirección 2 rueda derecha

//Configuración de los pines para la conexión con los Encoders
const int ENC_DER_C1 =  A0; //Encoders rueda derecha
const int ENC_DER_C2 =  A1;
const int ENC_IZQ_C1 =  A2; //Encoders rueda izquierda
const int ENC_IZQ_C2 =  A3;

//Configuracion de los pines de interrupcion de Obstáculos
const int INT_OBSTACULO = A4;

//Variables globales que se utilizan en el control de movimientos

//Variables para la máquina de estados principal
enum PosiblesEstados {AVANCE=0, GIRE_DERECHA, GIRE_IZQUIERDA,ESCOGER_DIRECCION,GIRO, NADA};
char *PosEstados[] = {"AVANCE", "GIRE_DERECHA", "GIRE_IZQUIERDA","ESCOGER_DIRECCION","GIRO", "NADA"};
PosiblesEstados estado = AVANCE;

//variable que almacena el tiempo del último ciclo de muestreo
long tiempoActual=0;

//contadores de pulsos del encoder, muy importantes
int contPulsosDerecha=0;
int contPulsosIzquierda=0;
int contPulsosDerPasado=0;
int contPulsosIzqPasado=0;

//Variables que almacenan el desplazamiento angular de cada rueda
float posActualRuedaDerecha=0.0;
float posActualRuedaIzquierda=0.0;

//Variables del valor de velocidad en cada rueda
float velActualDerecha=0.0;
float velActualIzquierda=0.0;

//Valores acumulados para uso en las ecuaciones de control
//Para Giro
float errorAnteriorGiroDer=0;
float errorAnteriorGiroIzq=0;
float sumErrorGiroDer=0;
float sumErrorGiroIzq=0;
int contCiclosEstacionarioGiro=0;
//Para Velocidad
float errorAnteriorVelDer=0;
float errorAnteriorVelIzq=0;
float sumErrorVelDer=0;
float sumErrorVelIzq=0;

//Variables para las interrupciones de los encoders
byte estadoEncoderDer=1; 
byte estadoEncoderIzq=1;

//Variables para el algoritmo de exploracion
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
  pinMode(PWMA, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(INT_OBSTACULO, INPUT_PULLUP);
  pinMode(13,OUTPUT);
  attachInterrupt(ENC_DER_C1, PulsosRuedaDerechaC1,CHANGE);  //conectado el contador C1 rueda derecha
  attachInterrupt(ENC_DER_C2, PulsosRuedaDerechaC2,CHANGE); 
  attachInterrupt(ENC_IZQ_C1, PulsosRuedaIzquierdaC1,CHANGE);  //conectado el contador C1 rueda izquierda
  attachInterrupt(ENC_IZQ_C2, PulsosRuedaIzquierdaC2,CHANGE);
  attachInterrupt(digitalPinToInterrupt(INT_OBSTACULO), DeteccionObjeto,FALLING);
  Serial.begin(9600);
  tiempoActual=micros();
  //Para el algoritmo de exploración
  randomSeed(analogRead(A5));
  delay(3000);
  //direccionDeseada=random(1,5);//asigna direccion N,S,E,O
}

void loop(){
  //Las acciones de la máquina de estados y los controles se efectuarán en tiempos fijos de muestreo
  if((micros()-tiempoActual)>=tiempoMuestreo){
     tiempoActual=micros();
    
     //Máquina de estados que cambia el modo de operación
     switch (estado) {
  
        case AVANCE:  { 
          bool avanceTerminado= AvanzarDistancia(100); 
          if (avanceTerminado || !digitalRead(INT_OBSTACULO)){
            ConfiguracionParar(); //detiene el carro un momento
            estado = ESCOGER_DIRECCION; 
          }
          break; 
        }
        case GIRE_DERECHA: { 
          bool giroTerminado= Giro(90);
          if   (giroTerminado) {
            ConfiguracionParar(); //detiene el carro un momento
            estado = GIRE_IZQUIERDA;
          }
          break; 
        }
        case GIRE_IZQUIERDA:  { 
          bool giroTerminado= Giro(-90);
          if   (giroTerminado) {
            ConfiguracionParar(); //detiene el carro un momento
            estado = GIRE_DERECHA;
          }        
          break; 
        }
        case ESCOGER_DIRECCION: {
          EscogerDireccion();
          estado= GIRO;
        }
        case GIRO: {
          bool giroTerminado=Giro(anguloGiro);
          if(giroTerminado){
            ConfiguracionParar();
            if(digitalRead(INT_OBSTACULO)==1){// no hay un obstáculo, el giro es exitoso.
              detectaObjeto=0;
              if(asignarDireccionDeseada==1){
                direccionDeseada=direccionActual;  
                asignarDireccionDeseada=0;
              }
              estado=AVANCE;
              ReinicioEstadosDisponiblesGiro();
            }
           else if(digitalRead(INT_OBSTACULO)==0){// hay un obstáculo, giro no es exitoso.
             estado=ESCOGER_DIRECCION;
           }
          }
        }
        case NADA: { 
          break; 
        }
     }
   
  }
 
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

    if(digitalRead(INT_OBSTACULO)==0){//hay un obstaculo frente
      //Si hay obstaculo opcionGiro se limita a 1=Izquierda,3=Derecha,5=Giro180
      //Utiliza las memorias de direccion Disponible, las cuales al avanzar regresan a 1.
      AsignaDireccionConObstaculo();
     }
    else if(digitalRead(INT_OBSTACULO)==1){//Si no hay obstaculo
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
   ConfiguracionParar();
   estado=ESCOGER_DIRECCION;
  } 
   
}


bool Giro(float grados){

    posActualRuedaDerecha= ConvDistAngular(contPulsosDerecha);
    int cicloTrabajoRuedaDerecha = ControlPosGiroRueda( grados, posActualRuedaDerecha, sumErrorGiroDer, errorAnteriorGiroDer );
    
    posActualRuedaIzquierda= ConvDistAngular(contPulsosIzquierda);
    int cicloTrabajoRuedaIzquierda = ControlPosGiroRueda( -grados, -posActualRuedaIzquierda, sumErrorGiroIzq, errorAnteriorGiroIzq);
    
    Serial.print(cicloTrabajoRuedaDerecha);
    Serial.print(",");
    Serial.println(cicloTrabajoRuedaIzquierda);
  
    ConfiguraEscribePuenteH (cicloTrabajoRuedaDerecha, cicloTrabajoRuedaIzquierda);

    bool giroListo = EstadoEstacionario (cicloTrabajoRuedaDerecha, cicloTrabajoRuedaIzquierda, contCiclosEstacionarioGiro);

    return giroListo;

} 

float ConvDistAngular(float cantPulsos){
//Calcula la distancia angular (en grados) que se ha desplazado el robot. 
//Para robots diferenciales, en base a un giro sobre su eje (no arcos)
//Recibe la cantidad de pulsos de la rueda. Devuelve un ángulo en grados.

  float despAngular=cantPulsos/(distanciaCentroARueda*pulsosPorMilimetro)*180/PI; 
  return despAngular;
}

int ControlPosGiroRueda( float posRef, float posActual, float& sumErrorGiro, float& errorAnteriorGiro ){
//Funcion para implementar el control PID por posición en una rueda. 
//Se debe tener un muestreo en tiempos constantes, no aparecen las constantes de tiempo en la ecuación, sino que se integran con las constantes Ki y Kd
//Se recibe la posición de referencia, la posición actual (medida con los encoders), el error acumulado (por referencia), y el valor anterior del error (por referencia)
//Entrega el valor de ciclo de trabajo (PWM) que se enviará al motor CD

  float errorGiro= posRef-posActual; //se actualiza el error actual

  //se actualiza el error integral y se restringe en un rango, para no aumentar sin control
  //como son variables que se mantienen en ciclos futuros, se usan variables globales
  sumErrorGiro += errorGiro; 
  sumErrorGiro = constrain(sumErrorGiro,errorMinIntegral,errorMaxIntegral);

  //error derivativo (diferencial)
  float difErrorGiro= (errorGiro - errorAnteriorGiro);

  //ecuación de control PID
  float pidTermGiro= (KpGiro*errorGiro) + (KiGiro * sumErrorGiro) + (KdGiro*difErrorGiro);

  //Se limita los valores máximos y mínimos de la acción de control, para no saturarla
  pidTermGiro= constrain( int (pidTermGiro),limiteInferiorCicloTrabajoGiro,limiteSuperiorCicloTrabajoGiro); 

  //Restricción para manejar la zona muerta del PWM sobre la velocidad
  if (-cicloTrabajoMinimo < pidTermGiro && pidTermGiro < cicloTrabajoMinimo){
    pidTermGiro = 0; 
  }
  
  //actualiza el valor del error para el siguiente ciclo
  errorAnteriorGiro = errorGiro;
  
  return  pidTermGiro; 
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

void ConfiguracionParar(){
  //Deshabilita las entradas del puente H, por lo que el carro se detiene
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, LOW); 
  ResetContadoresEncoders();
  delay(250);
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

bool EstadoEstacionario (int pwmRuedaDer, int pwmRuedaIzq, int& contCiclosEstacionario){
//Función que revisa el estado de la acción de control y si se mantiene varios ciclos en cero, asume que ya está en el estado estacionario
//Recibe los ciclos de trabajo en cada rueda y una variable por referencia donde se almacenan cuantos ciclos seguidos se llevan
//Devuelve una variable que es TRUE si ya se alcanzó el estado estacionario

    bool estadoEstacionarioAlcanzado= false;
    
    if(pwmRuedaDer==0 && pwmRuedaIzq==0){
       contCiclosEstacionario++;
       if (contCiclosEstacionario > minCiclosEstacionario){
          ResetContadoresEncoders();
          contCiclosEstacionario=0;
          estadoEstacionarioAlcanzado= true;
       }     
    }else{
      contCiclosEstacionario=0;
    }

  return estadoEstacionarioAlcanzado;
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


bool AvanzarDistancia(int distanciaDeseada){
//Avanza hacia adelante una distancia definida en mm a velocidad constante
//Devuelve true cuando alcanzó la distancia deseada

  velActualDerecha= calculaVelocidadRueda(contPulsosDerecha, contPulsosDerPasado);
  int cicloTrabajoRuedaDerecha = ControlVelocidadRueda(velRequerida, velActualDerecha, sumErrorVelDer, errorAnteriorVelDer);

  velActualIzquierda= -1.0 * calculaVelocidadRueda(contPulsosIzquierda, contPulsosIzqPasado); //como las ruedas están en espejo, la vel es negativa cuando avanza, por eso se invierte
  int cicloTrabajoRuedaIzquierda = ControlVelocidadRueda(velRequerida, velActualIzquierda, sumErrorVelIzq, errorAnteriorVelIzq);
   
    Serial.print("Vel der: ");
    Serial.print(velActualDerecha,5);
    Serial.print(" ; Vel izq: ");
    Serial.println(velActualIzquierda,5);

    Serial.print("PWM der: ");
    Serial.print(cicloTrabajoRuedaDerecha);
    Serial.print(" ; PWM izq: ");
    Serial.println(cicloTrabajoRuedaIzquierda);
  
  ConfiguraEscribePuenteH (cicloTrabajoRuedaDerecha, cicloTrabajoRuedaIzquierda);

  float distanciaAvanzada= calculaDistanciaLinealRecorrida();
  
  bool avanceListo = false; 
  if (distanciaAvanzada >= distanciaDeseada) {
    avanceListo = true; 
    ResetContadoresEncoders();
  }
  
  return avanceListo;
}

void AvanzarIndefinido(){
//Avanza hacia adelante indefinidamente
//Luego de utilizarse, se debe llamar a ResetContadoresEncoders()

  velActualDerecha= calculaVelocidadRueda(contPulsosDerecha, contPulsosDerPasado);
  int cicloTrabajoRuedaDerecha = ControlVelocidadRueda(velRequerida, velActualDerecha, sumErrorVelDer, errorAnteriorVelDer);

  velActualIzquierda= -1.0 * calculaVelocidadRueda(contPulsosIzquierda, contPulsosIzqPasado); //como las ruedas están en espejo, la vel es negativa cuando avanza, por eso se invierte
  int cicloTrabajoRuedaIzquierda = ControlVelocidadRueda(velRequerida, velActualIzquierda, sumErrorVelIzq, errorAnteriorVelIzq);
  
  ConfiguraEscribePuenteH (cicloTrabajoRuedaDerecha, cicloTrabajoRuedaIzquierda);

}

float calculaVelocidadRueda(int& contPulsos, int& contPulsosPasado){
//Función que calcula la velocidad de una rueda en base a la cantidad de pulsos del encoder y el tiempo de muestreo

    float velActual= ((contPulsos-contPulsosPasado) / pulsosPorMilimetro) / ((float)tiempoMuestreo/conversionMicroSaSDiv); //velocidad en mm por s
    contPulsosPasado= contPulsos;
    
    return velActual;
}

float calculaDistanciaLinealRecorrida(){
//Función que realiza el cálculo de la distancia lineal recorrida por cada rueda
//Devuelve el promedio de las distancias

  float distLinealRuedaDerecha= (float)contPulsosDerecha / pulsosPorMilimetro;
  float distLinealRuedaIzquierda= - (float)contPulsosIzquierda / pulsosPorMilimetro;
  float DistanciaLineal= (distLinealRuedaDerecha+distLinealRuedaIzquierda) /2.0;

  return DistanciaLineal;
}

int ControlVelocidadRueda( float velRef, float velActual, float& sumErrorVel, float& errorAnteriorVel ){
//Funcion para implementar el control PID por velocidad en una rueda. 
//Se debe tener un muestreo en tiempos constantes, no aparecen las constantes de tiempo en la ecuación, sino que se integran con las constantes Ki y Kd
//Se recibe la posición de referencia, la posición actual (medida con los encoders), el error acumulado (por referencia), y el valor anterior del error (por referencia)
//Entrega el valor de ciclo de trabajo (PWM) que se enviará al motor CD

  float errorVel= velRef-velActual; //se actualiza el error actual
   
  //se actualiza el error integral y se restringe en un rango, para no aumentar sin control
  //como son variables que se mantienen en ciclos futuros, se usan variables globales
  sumErrorVel += errorVel; 
  sumErrorVel = constrain(sumErrorVel,errorMinIntegral,errorMaxIntegral);

  //error derivativo (diferencial)
  float difErrorVel= (errorVel - errorAnteriorVel);

  //ecuación de control PID
  float pidTermVel= (KpVel*errorVel) + (KiVel * sumErrorVel) + (KdVel*difErrorVel);

  //Se limita los valores máximos y mínimos de la acción de control, para no saturarla
  pidTermVel= constrain( (int)pidTermVel,limiteInferiorCicloTrabajoVelocidad,limiteSuperiorCicloTrabajoVelocidad); 

//  //Restricción para manejar la zona muerta del PWM sobre la velocidad
//  if (-cicloTrabajoMinimo < pidTermVel && pidTermVel < cicloTrabajoMinimo){
//    pidTermVel = 0; 
//  }
      
  //actualiza el valor del error para el siguiente ciclo
  errorAnteriorVel = errorVel;
  
  return  ((int)pidTermVel); 
}
