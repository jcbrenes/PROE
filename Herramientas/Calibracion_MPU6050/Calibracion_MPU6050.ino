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
  for(int j=0;j<=2;j++){
    float gx_prom=0;
    float gy_prom=0;
    float gz_prom=0;
    float acx_prom=0;
    float acy_prom=0;
    float acz_prom=0;
    for (int i=0;i<=100;i++){
      float gxx,gyy,gzz,axx,ayy,azz;
      leeMPU(gxx, gyy, gzz, axx, ayy, azz);
      acx_prom=acx_prom+axx;
      acy_prom=acy_prom+ayy;
      acz_prom=acz_prom+azz;
      gx_prom=gx_prom+gxx;
      gy_prom=gy_prom+gyy;
      gz_prom=gz_prom+gzz;
    }
   gx_off=gx_off+(gx_prom/100);
   gy_off=gy_off+(gy_prom/100);
   gz_off=gz_off+(gz_prom/100);
   acx_off=acx_off+(acx_prom/100);
   acy_off=acy_off+(acy_prom/100);
   acz_off=acz_off+(16384-(acz_prom/100));
  }
  //Guarda los datos en la EEPROM
  guardarDatoFloat(gx_off,16);
  guardarDatoFloat(gy_off,20);
  guardarDatoFloat(gz_off,24);
  guardarDatoFloat(acx_off,28);
  guardarDatoFloat(acy_off,32);
  guardarDatoFloat(acz_off,36);
  Serial.println("Datos guardados con exito");
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

  gx=float(gyro_x)-gx_off;
  gy=float(gyro_y)-gy_off;
  gz=float(gyro_z)-gz_off;

  ax=float(ac_x)-acx_off;
  ay=float(ac_y)-acy_off;
  az=float(ac_z)+acz_off;
}


void guardarDatoFloat(float dato,int dirPagInicial){
    byte varr1; //primeros 8 bits para guardar en la EEPROM (LSB)
    byte varr2; //ultimo 8 bits para guardar en la EEPROM (MSB)
    byte varrDec; //parte decimal para guardar en la EEPROM
    byte signo; // signo del numero
    float dato2;
    if (dato<0.0){
      signo=1;
      dato2=(-1*dato);
    }
    else{
      signo=0;
      dato2=dato;
      } 
    transfVar(dato2,varr1,varr2,varrDec);
    eepromEscribe(dirEEPROM,dirPagInicial,varr1);
    delay(100);
    eepromEscribe(dirEEPROM,dirPagInicial+1,varr2);
    delay(100);
    eepromEscribe(dirEEPROM,dirPagInicial+2,varrDec);
    delay(100);
    eepromEscribe(dirEEPROM,dirPagInicial+3,signo);
    delay(100);
 }

float constrVar(byte LSB, byte MSB, byte dec){
    int piv1,piv2;
    float nuevo,decs;
    piv1=LSB;
    piv2=MSB<<8;
    nuevo=piv1|piv2;
    decs=float(dec);
    nuevo=nuevo+decs/100;
    return nuevo;
  }
void transfVar(float num,byte &var1,byte &var2,byte &varDec){
    int nuevoNum=int(num);
    int desplazamiento;
    float dec;
    desplazamiento=nuevoNum<<8;
    var1=desplazamiento>>8; //LSB
    var2=nuevoNum>>8; //MSB
    dec=100*(num-nuevoNum);
    varDec=int(dec);
  }

void eepromEscribe(byte dir, byte dirPag, byte data) {
  myWire.beginTransmission(dir);
  myWire.write(dirPag);
  myWire.write(data);
  myWire.endTransmission();
}

byte eepromLectura(int dir, int dirPag) {
  myWire.beginTransmission(dir);
  myWire.write(dirPag);
  myWire.endTransmission();

  myWire.requestFrom(dir, 1);
  if(myWire.available())
    return myWire.read();
  else
    return 0xFF;
}
