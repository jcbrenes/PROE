//Código principal para el proyecto PROE
//https://github.com/jcbrenes/PROE

#include <math.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_L3GD20_U.h>
#include <Adafruit_LSM303_U.h>
#include <MadgwickAHRS.h>

//Constantes del robot empleado
const float pi = 3.141592654;                   //Valor de pi.
const int tiempoMuestreo = 10000;               //Unidades: micro segundos
//const float pulsosPorRev = 206.0;             //Cantidad de pulsos de una única salida
//const int factorEncoder = 4;                  //Cantidad de tipos de pulsos que se están detectando (juego entre las 2 salidas del encoder)
const float conversionRadPulsos = 7625.224887;  //El valor viene de: 2pi*10^6/factorEncoder*pulsosPorRevUnidades. Unidades: urad/pulso. El micro es porque micros() da el tiempo en us.
//Mecánicas
const float xR_Chasis = -7.25; //Coordenada en el marco del robot del punto a controlar. Unidades cm. Medido desde el centro del eje de giro.
const float yR_Chasis = 0;   //Coordenada en el marco del robot del punto a controlar. Unidades cm. Medido desde el centro del eje de giro.
const float r_Rueda = 2.2;   //Radio de las ruedas del robot. Unidades cm.
const float d_Eje = 6.3;     //Distancias entre el centro del eje de giro y el punto de contacto de una rueda. Unidades cm.

//Constantes y variables para esquema de control de posición
const uint8_t K_Ganancia = 9;  //Constante de ganancia en el lazo de control de posición. Unidades 1/s.
float xPd = 0;                 //Coordenada X deseada del punto a controlar. Unidades cm.
float yPd = 0;                 //Coordenada Y deseada del punto a controlar. Unidades cm.
float xP = 0;                  //Coordenada X actual del punto a controlar. Unidades cm.
float yP = 0;                  //Coordenada Y actual del punto a controlar. Unidades cm.
float xPe = 0;                 //Error de posición. Unidades cm.
float yPe = 0;                 //Error de posición. Unidades cm.
float xP_dev = 0;              //Derivada de la coordenada X actual del punto a controlar. Unidades cm/s.
float yP_dev = 0;              //Derivada de la coordenada Y actual del punto a controlar. Unidades cm/s.
float v = 0;                   //Velocidad lineal del punto a controlar. Unidades cm/s.
float w = 0;                   //Velocidad angular del punto a controlar. Unidades rad/s.
float uL = 0;                  //Señal de control de velocidad angular para el motor izquierdo. Unidades rad/s.
float uR = 0;                  //Señal de control de velocidad angular para el motor derecho. Unidades rad/s.
float phi_dev = 0;             //Derivada del ángulo de orientación del robot. Unidades rad/s.
float phi = 0;                 //Ángulo de orientación actual del robot. Unidades rad.
float x_dev = 0;               //Derivada de la coordenada X actual del centro del eje de giro de las ruedas. Unidades cm/s.
float y_dev = 0;               //Derivada de la coordenada X actual del centro del eje de giro de las ruedas. Unidades cm/s.
int phi_ficticio = 0;          //PRUEEEEEEEBA: Ángulo teórico que debería tener el robot. Unidades rad.

//Constantes y variables para control PID
unsigned long currentTimeIzq, previousTimeIzq, currentTimeDer, previousTimeDer;  //Contadores de tiempo actual y pasado para cálculos de integrar y derivar en PID. Unidades us.
//Estas constantes se obtuvieron con 0.748 de velocidad de convergencia y 0.9 de robustez en el PIDTuner.
const float KpIzq = 2.412;                                                       //Constante KpIzq del control PID de velocidad del motor izquierdo.
const float KiIzq = 16.68;                                                       //Constante KiIzq del control PID de velocidad del motor izquierdo.
const float KdIzq = 0.06774;                                                     //Constante KdIzq del control PID de velocidad del motor izquierdo.
const float KpDer = 2.49;                                                        //Constante KpDer del control PID de velocidad del motor derecho.
const float KiDer = 17.96;                                                       //Constante KiDer del control PID de velocidad del motor derecho.
const float KdDer = 0.08631;                                                     //Constante KdDer del control PID de velocidad del motor derecho.
float elapsedTimeIzq, elapsedTimeDer;                                            //Contadores de tiempo actual y pasado para cálculos de integrar y derivar en PID. Unidades us.    
float errorIzq, lastErrorIzq, errorDer, lastErrorDer;                            //Variables para llevar los errores.
float referenciaIzq, referenciaDer;                                              //Variables para saber el valor de rad/s que debe seguir el PID.
float cicloTrabajoRuedaIzquierda, cicloTrabajoRuedaDerecha;                      //Variables para guardar el valor de PWM resultante del PID. Se manda a los motores.
float cumErrorIzq, rateErrorIzq, cumErrorDer, rateErrorDer;                      //Variables para llevar los errores. CumError es el integral y rateError el derivativo.
const int errorMinIntegral = -250;                                               //Contante que limita el error integral.
const int errorMaxIntegral = 250;                                                //Contante que limita el error integral.
const int limiteSuperiorCicloTrabajo = 200;                                      //Contante que limita el PWM.
const int limiteInferiorCicloTrabajo = -200;                                     //Contante que limita el PWM.
const int limiteMuertoCicloTrabajo = 30;                                         //Constante para saber cuándo se alcanza el estado estacionario.

//Constantes generales para la lógica de control
const int minCiclosEstacionario = 30;                                            //Contante para saber cuándo se está en estado estacionario.
const float gradosCorreccion = 45;                                               //Esta constante me permite saber cuándo Girar() se llamó para una corrección o para un giro de +-90°.
const float correccionControlPos = 0.8976;                                       //Esta constante escala la referencia de control y la estimación del robot para que el control opere bien.
                          //El 0.8976 se obtuvo luego de una serie de experimentos para lograr mejorar el dato de control.
                          //Se utiliza solo en avanzar y solo para las coordenadas xP y yP, ni el movimiento de giro ni la orientación
                          //necesitan esta corrección. Para la referencia se hace: ref*correccionControlPos, y para la estimación se
                          //usa el inverso: xP*(1/correccionControlPos); o bien: yP*(1/correccionControlPos).

//Variables de la lógica de la IMU
bool usarIMU = 0;                                                                //Variable que decide en el control si usar el cálculo de la IMU o no.
float imuActual = 0;                                                             //Medición de la IMU usada para calcular vel. angular con base en IMU.
float imuAnterior = 0;                                                           //Medición de la IMU usada para calcular vel. angular con base en IMU.
float imuOrientacionInicial = 0;                                                 //Valor inicial de la IMU. Es necesario para que el sensor inicie con orientación cero.
float phiOdom = 0;                                                               //Valor de Phi necesario para llevar el cálculo de la odometría independientemente de la IMU.

//Configuración de pines de salida para conexión con el Puente H
const int PWMA = 10; //Control velocidad izquierdo
const int AIN1 = 11; //Dirección 1 rueda izquierda 
const int AIN2 = 12; //Dirección 2 rueda izquierda 
const int PWMB = 9;  //Control velocidad derecha
const int BIN1 = 6;  //Dirección 1 rueda derecha
const int BIN2 = 5;  //Dirección 2 rueda derecha

//Configuración de los pines para la conexión con los Encoders
const int ENC_DER_C1 =  A0;  //Encoders rueda derecha
const int ENC_DER_C2 =  A1;
const int ENC_IZQ_C1 =  A2;  //Encoders rueda izquierda
const int ENC_IZQ_C2 =  A3;

//Configuracion de los pines de interrupcion de Obstáculos
const int INT_OBSTACULO = A4;

//Variables para la máquina de estados principal
enum PosiblesEstados {AVANCE = 0, GIRE_DERECHA, GIRE_IZQUIERDA, ESCOGER_DIRECCION, GIRO, NADA};
char *PosEstados[] = {"AVANCE", "GIRE_DERECHA", "GIRE_IZQUIERDA","ESCOGER_DIRECCION","GIRO", "NADA"};
PosiblesEstados estado = AVANCE;

//Variables de tiempo
long tiempoActual = 0;                     //Almacena el tiempo del último ciclo de muestreo.
long tiempoTranscurridoVelocidad = 0;      //Almacena el tiempo transcurrido entre un cálculo de velocidad de las ruedas y otro.
long tiempoTranscurridoEstimacionPos = 0;  //Almacena el tiempo transcurrido entre un cálculo de estimación de posición del robot móvil y otro.
long tiempoDerivacionIMU = 0;              //Almacena el tiempo transcurrio entre una medición de la IMU y la siguiente para calcular la velocidad angular del robot.

//Contadores de pulsos del encoder, muy importantes
int contPulsosDerecha = 0;
int contPulsosIzquierda = 0;

//Variables del valor de velocidad en cada rueda
float velRuedaIzq = 0; 
float velRuedaDer = 0;

//Variables para las interrupciones de los encoders
byte estadoEncoderDer = 1; 
byte estadoEncoderIzq = 1;

//Variables para el algoritmo de exploracion
bool giroAnteriorIzquierda = 0;
bool giroAnteriorDerecha = 0;
int direccionDeseada = 1;  //Random(1,5);//asigna direccion N,S,E,O 
int direccionActual = 1;   //1=N, 2=E, 3=S, 4=O
bool giroTerminado = 1;
bool detectaObjeto = 0;
float anguloGiro = 90.0;
bool direccionAdelanteDisponible = 1;
bool direccionIzquierdaDisponible = 1;
bool direccionDerechaDisponible = 1;
int opcionGiro = 0;  //Puede tener 3 valores, 1=Izquierda 2=Adelante, 3=Derecha
bool asignarDireccionDeseada = 0;

//Declaraciones de los drivers de sensores de la IMU
Adafruit_L3GD20_Unified       gyro(20);
Adafruit_LSM303_Accel_Unified accel(30301);
Adafruit_LSM303_Mag_Unified   mag(30302);

//Vecotr de compensación por el error hard iron
float mag_offsets[3]            = { 2.19F, -8.52F, 14.62F };

//Matriz 3x3 de compensación por el error soft iron
float mag_softiron_matrix[3][3] = { { 0.950, 0.008, 0.013 },
                                    { 0.008, 0.971, 0.006 },
                                    { 0.013, 0.006, 1.085 } };

//Vectores de offset para el giroscopio y acelerómetro
float gyro_offsets[3] = { 1.80209, -4.52367, 0.41807 };
float acc_offsets[3] =  { 0.20976, -0.13157, -0.31026 };

//Declaración del filtro Madgwick a utilizar
Madgwick filter;

void setup(){
  Serial.begin(9600);
  
  if(usarIMU){ //Si se usará la IMU, esperar a su inicialización.
    if(!gyro.begin()){
      Serial.println("No se detectó el L3GD20 (giro)");
      while(1);
    }
    if(!accel.begin()){
      Serial.println("No se detectó el L3M303DLHC (acc)");
      while(1);
    }
    if(!mag.begin()){
      Serial.println("No se detectó el L3M303DLHC (mag)");
      while(1);
    }
    filter.begin(36);  //Inicialización del filtro Madgwick. El filtro espera 36 muestras/sec. Este dato fue calculado según el tiempo de procesamiento del código.
    for(int contador = 1; contador <= 4300; contador++){  //Llamo al filtro 4300 veces (1 cada 28 ms, aprox. 120 segundos en total) para que este se estabilice antes de empezar el código.
      float datoParaEstabilizarFiltro = AnguloYaw();
      if (contador == 4300){imuOrientacionInicial = datoParaEstabilizarFiltro;} //Si es el último dato obtenido, guárdelo como el inicial de IMU.
      delay(28); //Hacer un delay de 28 ms asegura que se llame al filtro 36 veces por segundo, según lo espera él.
    }
  }

  //Definición de los pines de: control de motores y sensor de obstáculo
  pinMode(PWMA, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(INT_OBSTACULO, INPUT_PULLUP);
  pinMode(13,OUTPUT);

  //Ambos motores inician en bajo
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, HIGH);
  analogWrite(PWMA, 0);
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, HIGH);
  analogWrite(PWMB, 0);
  
  //Declaración de los vectores de interupcción
  attachInterrupt(ENC_DER_C1, PulsosRuedaDerechaC1,CHANGE);    //Conectado el contador C1 rueda derecha
  attachInterrupt(ENC_DER_C2, PulsosRuedaDerechaC2,CHANGE); 
  attachInterrupt(ENC_IZQ_C1, PulsosRuedaIzquierdaC1,CHANGE);  //Conectado el contador C1 rueda izquierda
  attachInterrupt(ENC_IZQ_C2, PulsosRuedaIzquierdaC2,CHANGE);
  //attachInterrupt(digitalPinToInterrupt(INT_OBSTACULO), DeteccionObjeto,FALLING);
  
  //Para el algoritmo de exploración
  randomSeed(analogRead(A5));
  
  delay(8000);
  //TODOS los contadores de tiempo deben ser refrescados justo antes de terminar el SetUp y TIENEN QUE refrescarse LUEGO del delay o habrá cálculos erróneos en el sistema de control.
  tiempoActual = micros();
  tiempoTranscurridoEstimacionPos = millis();
  tiempoTranscurridoVelocidad = micros();
  currentTimeIzq = micros();
  previousTimeIzq = micros();
  currentTimeDer = micros();
  previousTimeDer = micros();
  tiempoDerivacionIMU = millis();
}

void loop(){
//Las acciones de la máquina de estados y los controles se efectuarán en tiempos fijos de muestreo
//  if((micros()-tiempoActual) >= tiempoMuestreo){
//    tiempoActual = micros();
//    
//    //Máquina de estados que cambia el modo de operación
//    switch (estado){
//  
//       case AVANCE:{ 
//         bool avanceTerminado = Avanzar(10.0);
//         CorreccionAngulo();
//         if (avanceTerminado || !digitalRead(INT_OBSTACULO)){
//           ConfiguracionParar(); //detiene el carro un momento
//           estado = ESCOGER_DIRECCION; 
//         }
//         break; 
//       }
//       case GIRE_DERECHA:{ 
//         bool giroTerminado = Giro(90.0);
//         CorreccionAngulo();
//         if(giroTerminado){
//           ConfiguracionParar(); //detiene el carro un momento
//           estado = GIRE_IZQUIERDA;
//         }
//         break; 
//       }
//       case GIRE_IZQUIERDA:{ 
//         bool giroTerminado = Giro(-90.0);
//         CorreccionAngulo();
//         if(giroTerminado){
//           ConfiguracionParar(); //detiene el carro un momento
//           estado = GIRE_DERECHA;
//         }        
//         break; 
//       }
//       case ESCOGER_DIRECCION:{
//         EscogerDireccion();
//         estado = GIRO;
//       }
//       case GIRO:{
//         bool giroTerminado = Giro(anguloGiro);
//         CorreccionAngulo();
//         if(giroTerminado){
//           ConfiguracionParar();
//           if(digitalRead(INT_OBSTACULO) == 1){// no hay un obstáculo, el giro es exitoso.
//             detectaObjeto = 0;
//             if(asignarDireccionDeseada == 1){
//               direccionDeseada = direccionActual;  
//               asignarDireccionDeseada = 0;
//             }
//             estado = AVANCE;
//             ReinicioEstadosDisponiblesGiro();
//           }
//           else if(digitalRead(INT_OBSTACULO) == 0){// hay un obstáculo, giro no es exitoso.
//             estado = ESCOGER_DIRECCION;
//           }
//         }
//       }
//       case NADA:{ 
//         break; 
//       }
//    }
//  }

  Avanzar(15*correccionControlPos);  //Avanzar 15 cm, nótese la corrección de control. 
  CorreccionAngulo();                //Llamo al subsistema de vigilancia de ángulo.
  ConfiguracionParar();              //Paro los motores.
  //CorreccionIMU();                 //En caso de usar IMU, se llama a esta función en vez de al sistema de vigilancia de la orientación.
  //ConfiguracionParar();            //Paro los motores.
  
  for(int i = 1; i <= 900; i++){  //Esto es para que espere 90 segundos mientras imprime el error para poder ver los datos. Se puede borrar si no se necesita.
    Serial.print(xP*(1/correccionControlPos)); Serial.print(" ; ");  //Nótese que la estimación se corrige con el inverso de la constante.
    Serial.print(yP*(1/correccionControlPos)); Serial.print(" ; ");  //Nótese que la estimación se corrige con el inverso de la constante.
    Serial.println(phi*57.29578, 4);                                 //Paso de radianes a grados.

    Serial.println("----");
    delay(100);
  }
}

//void ReinicioEstadosDisponiblesGiro(){
//  direccionAdelanteDisponible = 1;
//  direccionDerechaDisponible = 1;
//  direccionIzquierdaDisponible = 1;
//}
//
//void AsignaDireccionSinObstaculo(){
//  if(direccionActual == direccionDeseada){
//    opcionGiro = random(1,4);  //1=izquierda 2=avanza 3=derecha
//  }
//  else if(direccionActual != direccionDeseada){
//    if(abs(direccionActual-direccionDeseada) == 2){  //Completamente en la direccion opuesta
//      opcionGiro = 4;
//    }
//    else if(giroAnteriorIzquierda == 1){
//      opcionGiro = random(2,4);  //Gire derecha o adelante
//    }
//    else if(giroAnteriorDerecha == 1){
//      opcionGiro = random(1,3);  //Gire izquierda o adelante
//    }
//    else if(giroAnteriorDerecha == 0 && giroAnteriorIzquierda == 0){
//      //Asume que siempre se coloca en direccion Norte
//      if(direccionDeseada == 2){  //Si la deseada es  Este
//        opcionGiro = 3;           //Gire a la izquierda
//      }
//      if(direccionDeseada == 4){  //Si la deseada es Oeste
//        opcionGiro = 1;           //Gire a la derecha
//      }
//    }
//  }
//
//
//}
//void AsignaDireccionConObstaculo(){
//  if(direccionActual == direccionDeseada){
//      direccionAdelanteDisponible = 0;
//      asignarDireccionDeseada = 1;
//      if(direccionDerechaDisponible == 1 && direccionIzquierdaDisponible == 1){
//        opcionGiro=random(1,3);//si es 1, gire izquierda
//        if(opcionGiro == 1){
//        }
//        if(opcionGiro == 2){//sino, gire derecha
//          opcionGiro = 3;
//        }
//      }
//      else if(direccionDerechaDisponible == 0 && direccionIzquierdaDisponible == 1){
//        opcionGiro = 1;
//      }
//      else if(direccionDerechaDisponible == 1 && direccionIzquierdaDisponible == 0){
//        opcionGiro = 3;
//      }
//      else if(direccionDerechaDisponible == 0 && direccionIzquierdaDisponible == 0 && direccionAdelanteDisponible == 0){
//        opcionGiro = 4;
//      }
//             
//  }
//  else if(direccionActual != direccionDeseada){
//    if(direccionAdelanteDisponible ==1){
//    switch(direccionDeseada){
//        case 1:
//          if(direccionActual == 4){
//             opcionGiro = 3;
//          }
//          if(direccionActual == 2){
//            opcionGiro = 1;
//          }
//          break;
//        case 4:
//          if(direccionActual == 1){
//            opcionGiro = 1;
//          }
//          if(direccionActual == 3){
//            opcionGiro = 3;
//          }
//          break;
//         case 2:
//          if(direccionActual == 1){
//            opcionGiro = 3;
//          }
//          if(direccionActual == 3){
//            opcionGiro = 1;
//          }
//          break;
//          case 3:
//          if(direccionActual == 2){
//            opcionGiro = 3;
//          }
//          if(direccionActual == 4){
//            opcionGiro = 1;
//           }
//          break;
//         default:
//          break;
//      }
//      if(opcionGiro == 1){
//        direccionDerechaDisponible = 0;
//      }
//      if(opcionGiro == 3){
//        direccionIzquierdaDisponible = 0;
//      }
//    
//  }
//  else if(direccionAdelanteDisponible == 0 && (direccionDerechaDisponible == 1 || direccionIzquierdaDisponible == 1)){
//    opcionGiro = 4;
//    if(giroAnteriorDerecha == 1){direccionIzquierdaDisponible = 0;}
//    if(giroAnteriorIzquierda == 1){direccionDerechaDisponible = 0;}
//  }
//  else{ 
//    switch(direccionDeseada){
//        case 1:
//          if(direccionActual == 4){
//             opcionGiro = 1;
//          }
//          if(direccionActual == 2){
//            opcionGiro = 3;
//          }
//          break;
//        case 4:
//          if(direccionActual == 1){
//            opcionGiro = 3;
//          }
//          if(direccionActual == 3){
//            opcionGiro = 1;
//          }
//          break;
//         case 2:
//          if(direccionActual == 1){
//            opcionGiro = 1;
//          }
//          if(direccionActual == 3){
//            opcionGiro = 3;
//          }
//          break;
//          case 3:
//          if(direccionActual == 2){
//            opcionGiro = 1;
//           }
//          if(direccionActual == 4){
//            opcionGiro = 3;
//           }
//          break;
//         default:
//          break;
//      }
//    }
//  }  
//}
//
//void EscogerDireccion(){
//    if(digitalRead(INT_OBSTACULO) == 0){//hay un obstaculo frente
//      //Si hay obstaculo opcionGiro se limita a 1=Izquierda,3=Derecha,5=Giro180
//      //Utiliza las memorias de direccion Disponible, las cuales al avanzar regresan a 1.
//      AsignaDireccionConObstaculo();
//     }
//    else if(digitalRead(INT_OBSTACULO) == 1){//Si no hay obstaculo
//      AsignaDireccionSinObstaculo();
//    }
//    switch (opcionGiro){
//      case 1: //Izquierda
//        giroAnteriorIzquierda = 1;
//        giroAnteriorDerecha = 0;
//        if(direccionActual == 1){direccionActual = 4;}
//        else {direccionActual--;}
//        anguloGiro = 90.0;
//        break;
//    
//      case 2: //Adelante
//        direccionActual = direccionActual;
//        anguloGiro = 0.0;
//        break;
//
//      case 3: //Derecha
//        giroAnteriorDerecha = 1;
//        giroAnteriorIzquierda = 0;
//        if(direccionActual == 4){direccionActual = 1;}
//        else{direccionActual++;}
//        anguloGiro = -90.0;
//        break;
//       case 4:
//        anguloGiro = 180.0;
//        direccionActual = direccionDeseada+2;
//        if(direccionActual == 5){direccionActual = 1;}
//        if(direccionActual == 6){direccionActual = 2;}
//        direccionDeseada = direccionActual;
//        giroAnteriorDerecha = 0;
//        giroAnteriorIzquierda = 0;
//        break;   
//      default:
//        break;
//    }
// 
//    if(direccionActual == 1){
//      Serial.print("^");
//    }
//    if(direccionActual == 2){
//      Serial.print(">");
//    }
//    if(direccionActual == 3){
//      Serial.print("v");
//    }
//    if(direccionActual == 4){
//      Serial.print("<");
//    }
//    Serial.print(" ");
//    if(direccionDeseada == 1){
//      Serial.print("^");
//    }
//    if(direccionDeseada == 2){
//      Serial.print(">");
//    }
//    if(direccionDeseada == 3){
//      Serial.print("v");
//    }
//    if(direccionDeseada == 4){
//      Serial.print("<");
//    }
//    Serial.print("   ");
//    if(direccionIzquierdaDisponible == 1){
//      Serial.print("<");
//    }
//   
//    if(direccionIzquierdaDisponible == 0){Serial.print("*");}
//    if(direccionAdelanteDisponible == 1){
//      Serial.print("^");
//    }
//    if(direccionAdelanteDisponible == 0){Serial.print("*");}
//    if(direccionDerechaDisponible == 1){
//      Serial.println(">");
//    }
//     if(direccionDerechaDisponible == 0){Serial.println("*");}
//     giroTerminado = 0;
//}
//
//void DeteccionObjeto(){
//  if(detectaObjeto == 0 && giroTerminado == 1){
//     detectaObjeto = 1;
//     Serial.println("OBJETO!!");
//     digitalWrite(13,HIGH);
//     ConfiguracionParar();
//     estado = ESCOGER_DIRECCION;
//  }  
//}

bool Giro(float grados){
  //Función que se encarga de generar los movimientos tipo arco de la navegación del robot móvil.
  
  Serial.print("Voy a girar: "); Serial.println(grados);
  //Verifico que la cantidad de grados a girar no es una corrección leve. Si es mayor a 60° se trata de un giro de 90°o 180°. El 60° fue escogido al azar. 
  if(abs(grados) > 60){                              
    phi_ficticio += (int)grados;  //Si sí es un giro de 90° o 180°, actualizo la orientación ficticia que debería tener el robot.
    Serial.print("Ángulo ficticio: "); Serial.println(phi_ficticio);
    if(abs(phi_ficticio) == 360){phi_ficticio = 0;}  //Al igual que el cálculo de la verdadera orientación del robot, cada vez que la orientación ficticia da una vuelta completa se actualiza.
  }
    
  float velAngular = 2*pi/8000;                             //Velocidad angular fija para los giros: 360° en 8 segundos, es decir, 2pi rad en 8000 ms. Unidades: rad/ms.
  //Las siguientes dos variables me permiten calcular el centro inicial de giro. Recordar que se controla un punto P
  //que NO es el centro de giro, pero al girar el robot debe saber cuál es el centro del círculo que va a trazar.
  float centroXInicial = xP - xR_Chasis*cos(phi);           //Coordenada X del centro del círculo a trazar.
  float centroYInicial = yP - xR_Chasis*sin(phi);           //Coordenada Y del centro del círculo a trazar.
  float orientacionInicial = phi;                           //Obtengo la orientación inicial respecto a la cual debo comenzar a girar.
  float gradRad = (float)grados*(pi/180);                   //Total de grados a girar en rad.
  float tiempoGiro = abs(gradRad)/velAngular;               //Tiempo en ms para completar el giro.
  int iteraciones = (int)tiempoGiro/(tiempoMuestreo/1000);  //Tiempo de muestreo está en us. Se pasa a ms (por eso los 1000) y se obtienen las iteraciones necesarias para completar el giro.
  
  //Este for es el que traza los puntos que componen la ruta de arco que seguirá el robot.
  for(int contador = 1; contador <= iteraciones; contador++){
    Serial.print("Girar "); Serial.println(contador);
    xPd = centroXInicial + xR_Chasis*cos(gradRad*((float)contador/iteraciones)+orientacionInicial);  //Nuevo X deseado según: el centro del círculo, la orientación inicial y la iteración en la que estoy.
    yPd = centroYInicial + xR_Chasis*sin(gradRad*((float)contador/iteraciones)+orientacionInicial);  //Nuevo Y deseado según: el centro del círculo, la orientación inicial y la iteración en la que estoy.

    //Lamo al control de posición.
    CalculoVelocidadesRobot();
    Mover();
    CalculoPosicionesRobot();
    delay(tiempoMuestreo/1000);  //Tiempo de muestreo en us, debo convertirlo a ms, por eso el 1000.
  }

  bool giroListo = EstadoEstacionario(true);  //Llamo a la función que se encarga de verificar si se llegó o no al estado estacionario.
  return giroListo;
}

bool Avanzar(float distancia){
  float velLineal = 3.0/1000;                                 //Velocidad lineal fija para el avance: 3 cm en 1 segundo, es decir, 3 cm en 1000 ms. Unidades: cm/ms.
  float tiempoAvance = distancia/velLineal;                   //Tiempo en ms para completar el giro.
  float orientacionInicial = phi;                             //Obtengo la orientación inicial sobre la cual debo de avanzar en línea recta.
  int iteraciones = (int)tiempoAvance/(tiempoMuestreo/1000);  //Tiempo de muestreo está en us. Se pasa a ms (por eso los 1000) y se obtienen las iteraciones necesarias para completar el giro. Se toma en cuenta que cada iteración dura 10 ms debido al tiempo de muestreo.
  
  for(int contador = 1; contador <= iteraciones; contador++){
    Serial.print("Avanzar "); Serial.println(contador);
    xPd = xPd + ((float)distancia/iteraciones)*cos(orientacionInicial);  //Calculo el nuevo Xp deseado con base en la posición anterior (el xPd anterior) y el incremento que debo hacer en la respectiva componente.
    yPd = yPd + ((float)distancia/iteraciones)*sin(orientacionInicial);  //Calculo el nuevo Yp deseado con base en la posición anterior (el yPd anterior) y el incremento que debo hacer en la respectiva componente.

    Serial.println(millis());
    //Llamo al control de posición.
    CalculoVelocidadesRobot();
    Mover();
    CalculoPosicionesRobot();
    delay(tiempoMuestreo/1000); //Tiempo de muestreo en us, debo convertirlo a ms, por eso los 1000.
  }
  
  bool avanceListo = EstadoEstacionario(false);  //Llamo a la función que se encarga de verificar si se llegó o no al estado estacionario.
  return avanceListo;
}

bool EstadoEstacionario(bool girar){
  //Función que revisa el estado de la acción de control y si se mantiene varios ciclos en las condiciones deseadas, 
  //asume que ya está en el estado estacionario. Las condiciones deseadas son: tanto para avanzar como girar, que la
  //referencia de velocidad a seguir por los motores sea cero; si el movimiento es girar, que la orientación tenga un 
  //error menor a 5°, y si es avanzar, que las coordenadas tengan un error menor a 0.5 cm.
  //Recibe una bandera booleana que le indica si el movimiento a supervisar es un giro o un avance.
  //Devuelve una variable que es TRUE si ya se alcanzó el estado estacionario del movimiento respectivo.

  bool estadoEstacionarioAlcanzado = false;
  int contCiclosEstacionario = 0;
    
  while(contCiclosEstacionario <= minCiclosEstacionario){
    if(girar){  //Se verifica si el movimiento era un giro, si sí, las condiciones toman en cuenta la orientación.
      if( ((abs(referenciaIzq) == 0) && (abs(referenciaDer) == 0)) || (abs(phi_ficticio - phi*57.29578) <= 5) ){  //Paso la orientación del robot a grados y evalúo que el error entre la estimación y la orientación teórica que debería tener el robot sea menor a 5°.
        contCiclosEstacionario++;    
      }
      else{
        contCiclosEstacionario = 0;
      }
    }
    else{  //Si no era un giro, sino que es avanzar, las condiciones toman en cuenta la posición.      
      if( ((abs(referenciaIzq) == 0) && (abs(referenciaDer) == 0)) || ((abs(xPd - xP) <= 0.5) && (abs(yPd - yP) <= 0.5)) ){  //Evalúo que el error en ambas coordenadas sea menor a 0.5 cm.
        contCiclosEstacionario++;    
      }
      else{
        contCiclosEstacionario = 0;
      }
    }

    //Llamo al control en cada iteración.
    Serial.print("Estacionario "); Serial.println(contCiclosEstacionario);
    CalculoVelocidadesRobot();
    Mover();
    CalculoPosicionesRobot();
    delay(tiempoMuestreo/1000);  //Tiempo de muestreo en us, debo convertirlo a ms, por eso los 1000.
  }
  
  estadoEstacionarioAlcanzado = true;
  return estadoEstacionarioAlcanzado;
}

void CalculoVelocidadesRobot(){
  //Función de cinemática inversa. 
  //Con el error en cada coordenada a controlar, obtiene la 
  //velocidad angular deseada en los motores. 
  //Unidades en la salida: rad/s.
  
  //Obtengo el error y lo escalo.
  xPe = K_Ganancia*(xPd - xP);
  yPe = K_Ganancia*(yPd - yP);
  Serial.print("Error xP: ");
  Serial.println(xPd - xP);
  Serial.print("Error yP: ");
  Serial.println(yPd - yP);

  //Obtengo las velocidades lineales y angulares del centro de giro del robot móvil.
  v = (cos(phi) - (yR_Chasis/xR_Chasis)*sin(phi))*xPe + (sin(phi) + (yR_Chasis/xR_Chasis)*cos(phi))*yPe;
  w = (-1*sin(phi)/xR_Chasis)*xPe + (cos(phi)/xR_Chasis)*yPe;
  
  
  //Obtengo las velocidades angulares en cada rueda del robot.
  uL = ((v-w*d_Eje)/r_Rueda); //unidades rad/s.
  uR = ((v+w*d_Eje)/r_Rueda); //unidades rad/s.
  
  //Limito la velodicad. La referencia máxima es de +-7 rad/s, el límite inferior para el cual la velocidad pasa a ser 0 es +-0.7 rad/s.
  uL = constrain(uL, -7.00, 7.00);
  if(-0.70 < uL && uL < 0.70){
    uL = 0.00;
  }
  uR = constrain(uR, -7.00, 7.00);
  if(-0.70 < uR && uR < 0.70){
    uR = 0.00;
  }
  
  Serial.print("uL: ");
  Serial.println(uL);
  Serial.print("uR: ");
  Serial.println(uR);

  //Guardo las velocidades angulares a seguir.
  referenciaIzq = uL;
  referenciaDer = uR;
}

void Mover(){
  //Función que muestrea el filtro de Madgwick y manipula el controlador PID.
  
  //Si estoy usando la IMU, aquí se rmuestrea con el filtro para tener la frecuencia de 36 Hz.
  if(usarIMU){AnguloYaw();}

  //Manipulación de los controladores PID.
  Velocidad();                                                                     //Llamo a la función que mide la velocidad en las ruedas con los encoders.
  cicloTrabajoRuedaIzquierda = PIDIzq(velRuedaIzq);                                //Llamo al controlador PID de la rueda izquierda.
  cicloTrabajoRuedaDerecha = PIDDer(velRuedaDer);                                  //Llamo al controlador PID de la rueda derecha.
  ConfiguraEscribePuenteH (cicloTrabajoRuedaDerecha, cicloTrabajoRuedaIzquierda);  //Llamo a la función que configura la dirección de giro de los motores según signo del PWM retornado por PID
}

void Velocidad(){
  //Esta función calcula la velocidad angular en cada una de las ruedas. Unidades: rad/s.
  long deltaT = micros() - tiempoTranscurridoVelocidad;                        //Obtengo el tiempo transcurrido.
  velRuedaIzq = (float)(-1*contPulsosIzquierda*(conversionRadPulsos/deltaT));  //Calculo velocidad rueda izquierda. Unidades rad/s.
  velRuedaDer = (float)(contPulsosDerecha*(conversionRadPulsos/deltaT));       //Calculo velocidad rueda izquierda. Unidades rad/s.
  Serial.print("Vel. Izq: ");
  Serial.println(velRuedaIzq);
  Serial.print("Vel. Der: ");
  Serial.println(velRuedaDer);
  
  //Reseteo los contadores de los encoders para el próximo cálculo.
  contPulsosIzquierda = 0;
  contPulsosDerecha = 0;

  //Guardo el tiempo actual para el próximo cálculo.
  tiempoTranscurridoVelocidad = micros();
}

float PIDIzq(float medicionIzq){
  //Función que hace el procesamiento del controlador PID de la rueda izquierda. 
  currentTimeIzq = micros();                                   //Obtengo el tiempo actual.
  elapsedTimeIzq = (float)(currentTimeIzq - previousTimeIzq);  //Calculo el tiempo transcurrido desde la iteración anterior
  elapsedTimeIzq *= 0.000001;                                  //Convierto de microsegundos a segundos. 
  
  errorIzq = referenciaIzq - medicionIzq;                                    //Determino el error. referenciaIzq viene de la función CalculoVelocidadesRobot() y medicioIzq de Velocidad().
  cumErrorIzq += errorIzq * elapsedTimeIzq;                                  //Calculo la integral del error.
  cumErrorIzq = constrain(cumErrorIzq, errorMinIntegral, errorMaxIntegral);  //Limito el error integral para que no crezca exponencialmente. Esto se debería limitar aún más.
  rateErrorIzq = (errorIzq - lastErrorIzq)/elapsedTimeIzq;                   //Calculo la derivada del error.

  float outIzq = KpIzq*errorIzq + KiIzq*cumErrorIzq + KdIzq*rateErrorIzq;    //Cálculo final del PID, el dato está en "unidades" de PWM.               
  
  lastErrorIzq = errorIzq;           //Guardo el error actual para el próximo cálculo.
  previousTimeIzq = currentTimeIzq;  //Guardo el tiempo actual para el próximo cálculo.

  if(outIzq > limiteSuperiorCicloTrabajo){outIzq = limiteSuperiorCicloTrabajo;}       //Limito la salida del controlador a no más de 200 PWM.
  else if(outIzq < limiteInferiorCicloTrabajo){outIzq = limiteInferiorCicloTrabajo;}  //Limito la salida del controlador a no más de -200 PWM.
  
  Serial.print("PID_I: ");
  Serial.println(outIzq);
  return outIzq;  //Retorno la salida del PID.
}

float PIDDer(float medicionDer){
  //Función que hace el procesamiento del controlador PID de la rueda derecha.     
  currentTimeDer = micros();                                   //Obtengo el tiempo actual.
  elapsedTimeDer = (float)(currentTimeDer - previousTimeDer);  //Calculo el tiempo transcurrido desde la iteración anterior
  elapsedTimeDer *= 0.000001;                                  //Convierto de microsegundos a segundos. 
  
  errorDer = referenciaDer - medicionDer;                                    //Determino el error. referenciaDer viene de la función CalculoVelocidadesRobot() y medicioDer de Velocidad().
  cumErrorDer += errorDer * elapsedTimeDer;                                  //Calculo la integral del error.
  cumErrorDer = constrain(cumErrorDer, errorMinIntegral, errorMaxIntegral);  //Limito el error integral para que no crezca exponencialmente. Esto se debería limitar aún más.
  rateErrorDer = (errorDer - lastErrorDer)/elapsedTimeDer;                   //Calculo la derivada del error.

  float outDer = KpDer*errorDer + KiDer*cumErrorDer + KdDer*rateErrorDer;    //Cálculo final del PID, el dato está en "unidades" de PWM.               
   
  lastErrorDer = errorDer;           //Guardo el error actual para el próximo cálculo.
  previousTimeDer = currentTimeDer;  //Guardo el tiempo actual para el próximo cálculo.

  if(outDer > limiteSuperiorCicloTrabajo){outDer = limiteSuperiorCicloTrabajo;}       //Limito la salida del controlador a no más de 200 PWM.
  else if(outDer < limiteInferiorCicloTrabajo){outDer = limiteInferiorCicloTrabajo;}  //Limito la salida del controlador a no más de -200 PWM.

  Serial.print("PID_d: ");
  Serial.println(outDer);
  return outDer;  //Retorno la salida del PID.
}

void ConfiguraEscribePuenteH(float pwmRuedaDer, float pwmRuedaIzq){
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
  delay(500);
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

void CalculoPosicionesRobot(){
  //Función de cinemática directa. 
  //Una vez ejecutado el control PID, con la medición
  //de los encoders calculo la posición actual del robot. 
  //Unidades en la salida: cm y rad.
  
  //Calculo la derivada de la orientación.
  phi_dev = (r_Rueda/(2*d_Eje))*(velRuedaDer - velRuedaIzq);
  Serial.print("phiDev: ");
  Serial.println(phi_dev);
  
  //Calculo las derivadas de las coordenadas del centro de rotación.
  x_dev = (r_Rueda*0.5*cos(phi))*(velRuedaDer + velRuedaIzq);
  y_dev = (r_Rueda*0.5*sin(phi))*(velRuedaDer + velRuedaIzq);

  //Calculo las derivadas de las coordenadas del punto a controlar.
  xP_dev = x_dev - phi_dev*(sin(phi)*xR_Chasis + cos(phi)*yR_Chasis);
  yP_dev = y_dev + phi_dev*(cos(phi)*xR_Chasis - sin(phi)*yR_Chasis);

  //Integro las derivadas de la posición del punto P y la orientación del robot para obtener los datos actuales.
  xP = xP + xP_dev*(millis() - tiempoTranscurridoEstimacionPos)/1000;     //Los 1000 son para pasar de ms a s.
  Serial.print("xP: ");
  Serial.println(xP);
  yP = yP + yP_dev*(millis() - tiempoTranscurridoEstimacionPos)/1000;     //Los 1000 son para pasar de ms a s.
  Serial.print("yP: ");
  Serial.println(yP);
  phi = phi + phi_dev*(millis() - tiempoTranscurridoEstimacionPos)/1000;  //Los 1000 son para pasar de ms a s.
  
  if(abs(phi) > 2*pi){phi = 0;}  //El control no sabe cuándo se da una vuelta completa, por lo que hay que resetearlo manualmente si se detecta esto.
  Serial.print("phi: ");
  Serial.println(phi);
  Serial.println("----------");
  
  //Guardo el tiempo actual para el próximo cálculo. 
  tiempoTranscurridoEstimacionPos = millis(); 
}

void CorreccionAngulo(){
  //Función que realiza la vigilancia de la orientación final luego de cada movimiento.
  Serial.println("Corrigiendo angulo");
  float phi_grad = phi*(180/pi);                     //Convierto la orientación del robot a grados.
  float phi_error = (float)phi_ficticio - phi_grad;  //Obtengo el error entre el ángulo que debería tener el robot ("ficticio") y el que realmente tiene.
  Giro(phi_error);                                   //Llamo a la función que realiza giros para que elimine el error.
}

void CorreccionIMU(){
  //Función que integra las funcionalidades de la IMU al sistema de control.
  Serial.println("IMU en acción");
  int contCiclosEstacionario = 0;
    
  while(contCiclosEstacionario <= 15){             //El ciclo se repite hasta que se logre obtener una medición positiva 15 veces seguidas.
    //Calculo la orientación del robot restándole a la medición actual de la IMU la primera medición.
    //Recordar que la IMU da un valor puntual y no un desplazamiento angular, por eso la resta.
    float imuMedicion = AnguloYaw() - imuOrientacionInicial;                                                       
    //if(imuMedicion < 0){imuMedicion += 360;}     //Normalizo. Esto debe estudiarse más, porque AQUÍ es donde está uno de los dos bugs el código.
    if( abs(phi_ficticio - imuMedicion) <= 1.5 ){  //Evalúo que el error entre la orientación actual medida con la IMU y la que debería tener el robot sea menor a 1.5°.
      contCiclosEstacionario++;                    //Si se cumple, se le suma uno a al contador de estado estacionario de la IMU.
    }
    else{
      Giro(phi_ficticio - imuMedicion);            //Si no cumple, se setea a 0 el contador para empezar desde el inicio y se llama a la función de giro con el error existente.
      contCiclosEstacionario = 0;
    }
  }
}

float AnguloYaw(){
  //Función que obtiene la orientación de la IMU/Filtro.
  
  //Se generan los eventos de los sensores.
  sensors_event_t gyro_event;
  sensors_event_t accel_event;
  sensors_event_t mag_event;

  //Se obtienen las nuevas mediciones de los sensores.
  gyro.getEvent(&gyro_event);
  accel.getEvent(&accel_event);
  mag.getEvent(&mag_event);

  //Hago la corrección de offset o hard iron a las lecturas del magnetómetro.
  float x = mag_event.magnetic.x - mag_offsets[0];
  float y = mag_event.magnetic.y - mag_offsets[1];
  float z = mag_event.magnetic.z - mag_offsets[2];

  //Aplico corrección de soft iron al magnetómetro.
  float mx = x * mag_softiron_matrix[0][0] + y * mag_softiron_matrix[0][1] + z * mag_softiron_matrix[0][2];
  float my = x * mag_softiron_matrix[1][0] + y * mag_softiron_matrix[1][1] + z * mag_softiron_matrix[1][2];
  float mz = x * mag_softiron_matrix[2][0] + y * mag_softiron_matrix[2][1] + z * mag_softiron_matrix[2][2];

  //La biblioteca del filtro espera la información del giro en °/s
  //pero el sensor de adafruit lo da en rad/s, entonces se debe
  //hacer la conversión.
  //Además, se aplican los offsets correspondientes a las mediciones.
  float gx = gyro_event.gyro.x * 57.29578F + gyro_offsets[0];
  float gy = gyro_event.gyro.y * 57.29578F + gyro_offsets[1];
  float gz = gyro_event.gyro.z * 57.29578F + gyro_offsets[2];

  //Le hago update al filtro. Nótese que ahí mismo aplico el offset a las mediciones del acelerómetro.
  filter.update(gx, gy, gz,
                accel_event.acceleration.x + acc_offsets[0], accel_event.acceleration.y + acc_offsets[1], accel_event.acceleration.z + acc_offsets[2],
                mx, my, mz);

  //Obtengo y retorno el valor d ela orientación estimado por el filtro.
  float heading = filter.getYaw();
  return heading;
}


//INTERRUPCIONES
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
