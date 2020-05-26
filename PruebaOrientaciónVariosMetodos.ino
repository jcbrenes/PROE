//En este documento se plantea la determinación de la orientación en 3 dimensiones utilizando distintos métodos.
//Cabe destacar que los métodos individualmente funcionan correctamente, Sólo utilizando el filtro de Madgwick, o sólo utilizando el filtro complementario
//Sin embargo, en esta implementación se presenta errores por velocidad de actualización de datos. Probar comentar secciones de código y dejar sólo uno de los métodos funcionando para ver su adecuado funcionamiento.
//Se presenta esta programación como guía para los cálculos a desarrollar únicamente.
#include "I2Cdev.h"
#include "MPU6050.h"
#include "Wire.h"
#include <MechaQMC5883.h>
MechaQMC5883 qmc;
#include <MadgwickAHRS.h>
Madgwick filter; 
const int mpuAddress = 0x68;  // Puede ser 0x68 o 0x69
MPU6050 mpu(mpuAddress);
 
int ax, ay, az;
int gx, gy, gz;
 
long tiempo_prev;
float dt;
float ang_x, ang_y,ang_z;
float ang_x_prev, ang_y_prev,ang_z_prev;
long gyro_x_cal, gyro_y_cal, gyro_z_cal,accel_x_cal,accel_y_cal;
float n=0.85;

float declinacion = -2.0;
float direccion = 0.0;
float mx, my, mz;
float min_x, max_x, mid_x;
float min_y, max_y, mid_y;
float min_z, max_z, mid_z;
float compass_rad2degree = 180.0 / PI;
float compass_y_scalled = 0;
float compass_x_scalled = 0;
float bearing = 0;

unsigned long microsPerReading, microsPrevious;

 
void setup()
{
   Serial.begin(9600);
   Wire.begin();
   mpu.initialize();
   Serial.println(mpu.testConnection() ? F("IMU iniciado correctamente") : F("Error al iniciar IMU"));
   for (int cal_int = 0; cal_int < 2000 ; cal_int ++){                  //Run this code 2000 times
    mpu.getRotation(&gx, &gy, &gz);  
    mpu.getAcceleration(&ax, &ay, &az);//Read the raw acc and gyro data from the MPU-6050
    accel_x_cal +=ax;
    accel_y_cal +=ay;
    gyro_x_cal += gx;                                              //Add the gyro x-axis offset to the gyro_x_cal variable
    gyro_y_cal += gy;                                              //Add the gyro y-axis offset to the gyro_y_cal variable
    gyro_z_cal += gz;                                              //Add the gyro z-axis offset to the gyro_z_cal variable
    delay(3);                                                          //Delay 3us to simulate the 250Hz program loop
    
  }
  accel_x_cal /=2000;
  accel_y_cal /=2000;
  gyro_x_cal /= 2000;                                                  //Divide the gyro_x_cal variable by 2000 to get the avarage offset
  gyro_y_cal /= 2000;                                                  //Divide the gyro_y_cal variable by 2000 to get the avarage offset
  gyro_z_cal /= 2000;
  qmc.init();
  microsPerReading = 1000000 / 25;
  microsPrevious = micros();
  filter.begin(25);
}
 
void loop() 
{
   unsigned long microsNow;
   float roll, pitch, heading;
   mpu.getAcceleration(&ax, &ay, &az);
   mpu.getRotation(&gx, &gy, &gz);
   int x, y, z;
   int azimuth;
   qmc.read(&x, &y, &z, &azimuth);
   compass_y_scalled = float(y) + 867.0;
   compass_x_scalled = 1.01045 * (float(x) + 107.0);
   Compass_Heading();
   ax-=accel_x_cal;
   ay-=accel_y_cal;
   gx-=gyro_x_cal;
   gy-=gyro_y_cal;
   gz-=gyro_z_cal;
   microsNow = micros();
   if (microsNow - microsPrevious >= microsPerReading) {
   // Leer las aceleraciones y velocidades angulares
   
    filter.updateIMU(gx/131.0, gy/131.0, gz/131.0, ax/16384.0, ay/16384.0, az/16384.0);
  //filter.update(gx, gy, gz, ax, ay, az,compass_x_scalled,compass_y_scalled,miz);
  // print the heading, pitch and roll
     roll = filter.getRoll();
     pitch = filter.getPitch();
     heading = filter.getYaw();
     updateFiltered();
     microsPrevious = microsPrevious + microsPerReading;
  }
   
   Serial.print(F("Madgwick roll: "));
   Serial.print(roll);
   Serial.print(F("\t Pitch:"));
   Serial.print(pitch);
   Serial.print(F("\t Yaw:"));
   Serial.print(heading);
   Serial.print(F("\t Rotacion en X:  "));
   Serial.print(ang_x);
   Serial.print(F("\t Rotacion en Y: "));
   Serial.print(ang_y);
   Serial.print(F("\t Rotacion en Z: "));
   Serial.print(ang_z);
   Serial.print(" ");
   Serial.print(F("\tYaw mag:"));
   Serial.println(bearing);
}
void Compass_Heading() {


  if (compass_y_scalled > 0) {
    bearing = 90 - atan(compass_x_scalled*cos(ang_y*(PI/180)) / compass_y_scalled*cos(ang_x*(PI/180))) * compass_rad2degree;
  } else if (compass_y_scalled < 0) {
    bearing = 270 - atan(compass_x_scalled*cos(ang_y*(PI/180)) / compass_y_scalled*cos(ang_x*(PI/180))) * compass_rad2degree;
  } else if (compass_y_scalled == 0 & compass_x_scalled < 0) {
    bearing = 180;
  } else {
    bearing = 0;
  }
}

void updateFiltered()
{
   dt = (millis() - tiempo_prev) / 1000.0;
   tiempo_prev = millis();
 
   //Calcular los ángulos con acelerometro
   float accel_ang_x = atan(ay / sqrt(pow(ax, 2) + pow(az, 2)))*(180.0 / 3.14);
   float accel_ang_y = atan(-ax / sqrt(pow(ay, 2) + pow(az, 2)))*(180.0 / 3.14);
 
   //Calcular angulo de rotación con giroscopio y filtro complementario
   ang_x = n*(ang_x_prev + (gx / 131)*dt) + (1-n)*accel_ang_x;
   ang_y = n*(ang_y_prev + (gy / 131)*dt) + (1-n)*accel_ang_y;
   ang_z =ang_z_prev + (gz / 131)*dt;
   ang_x_prev = ang_x;
   ang_y_prev = ang_y;
   ang_z_prev = ang_z;
   
}
