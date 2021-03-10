#include<Wire.h> //Bilbioteca para la comunicacion I2C
#include "wiring_private.h" // pinPeripheral() function
 
TwoWire myWire(&sercom1, 11, 13);

#define dirEEPROM B01010000 //Direccion de la memoria EEPROM
#define addr 0x0D //I2C Address para el HMC5883

//Radios de conversión según data sheet del MPU6050
#define A_R 16384.0
#define G_R 131.0

float gx_off,gy_off,gz_off,acx_off,acy_off,acz_off;

void setup(){
    Serial.begin(115200);
    myWire.begin();
    pinPeripheral(11, PIO_SERCOM);
    pinPeripheral(13, PIO_SERCOM);
    inicializarMPU();
    delay(1000);  
  }

void loop(){  
  for(int j=0;j<=1;j++){
    double gx_prom,gy_prom,gz_prom,acx_prom,acy_prom,acz_prom=0;
    for (int i=0;i<=100;i++){
      float gxx,gyy,gzz,axx,ayy,azz;
      leeMPU(gxx, gyy, gzz, axx, ayy, azz);
      acx_prom=acx_prom+axx;
      acy_prom=acy_prom+ayy;
      acz_prom=acz_prom+azz;
      gx_prom=gx_prom+gxx;
      gy_prom=gy_prom+gyy;
      gz_prom=gz_prom+gzz;
      Serial.print("gx: "); Serial.print(gxx); Serial.print(" gy: "); Serial.print(gyy); Serial.print(" gz: "); Serial.println(gzz);
      Serial.print("acx: "); Serial.print(axx); Serial.print(" acy: "); Serial.print(ayy); Serial.print(" acz: "); Serial.println(azz);
      delay(100);
    }
   gx_off=gx_off+(gx_prom/100);
   gy_off=gy_off+(gy_prom/100);
   gz_off=gz_off+(gz_prom/100);
   acx_off=acx_off+(acx_prom/100);
   acy_off=acy_off+(acy_prom/100);
   acz_off=acz_off+(16384-(acz_prom/100));
   Serial.println("Offsets actuales: ");
   Serial.print("gx_off: "); Serial.print(gx_off); Serial.print(" gy_off: "); Serial.print(gy_off); Serial.print(" gz:off: "); Serial.println(gz_off);
   Serial.print("acx_off: "); Serial.print(acx_off); Serial.print(" acy_off: "); Serial.print(acy_off); Serial.print(" acz_off: "); Serial.println(acz_off);
   delay(3000);
  }
  exit(0);
}

void inicializarMPU(){
  myWire.beginTransmission(0x68); //empezar comunicacion con el mpu6050
  myWire.write(0x6B);   //escribir en la direccion 0x6B
  myWire.write(0x00);   //escribe 0 en la direccion 0x6B (arranca el sensor)
  myWire.endTransmission();   //termina escritura
  }

//Funcione que extrae los datos crudos del MPU
void leeMPU(float &gx,float &gy,float &gz,float &ax,float &ay,float &az){
  
  int16_t gyro_x, gyro_y, gyro_z, tmp, ac_x, ac_y, ac_z; //datos crudos
  
  myWire.beginTransmission(0x68);   //empieza a comunicar con el mpu6050
  myWire.write(0x3B);   //envia byte 0x43 al sensor para indicar startregister
  myWire.endTransmission();   //termina comunicacion
  myWire.requestFrom(0x68,14); //pide 6 bytes al sensor, empezando del reg 43 (ahi estan los valores del giro)

  ac_x = myWire.read()<<8 | myWire.read();
  ac_y = myWire.read()<<8 | myWire.read();
  ac_z = myWire.read()<<8 | myWire.read();

  tmp = myWire.read()<<8 | myWire.read();

  gyro_x = myWire.read()<<8 | myWire.read(); //combina los valores del registro 44 y 43, desplaza lo del 43 al principio
  gyro_y = myWire.read()<<8 | myWire.read();
  gyro_z = myWire.read()<<8 | myWire.read();

  gx=gyro_x-gx_off;
  gy=gyro_y-gy_off;
  gz=gyro_z-gz_off;

  ax=ac_x-acx_off;
  ay=ac_y-acy_off;
  az=ac_z+acz_off;
}
