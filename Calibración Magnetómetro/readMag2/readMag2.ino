#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>

/* Assign a unique ID to this sensor at the same time */
Adafruit_LSM303_Mag_Unified mag = Adafruit_LSM303_Mag_Unified(12345);

//Correction values
const float rawLowX= -66.27; 
const float rawHighX= 68.27;
const float rawRangeX= abs(rawHighX)+abs(rawLowX);

const float rawLowY= -73.45; 
const float rawHighY= 60.64;
const float rawRangeY= abs(rawHighY)+abs(rawLowY);

const float referenceLow= -45; 
const float referenceHigh= 45;
const float referenceRange= 90;

void setup(void) 
{
  Serial.begin(9600);
  Serial.println("Magnetometer Test"); Serial.println("");
  
  /* Initialise the sensor */
  if(!mag.begin())
  {
    /* There was a problem detecting the LSM303 ... check your connections */
    Serial.println("Ooops, no LSM303 detected ... Check your wiring!");
    while(1);
  }
}

void loop(void) 
{
  /* Get a new sensor event */ 
  sensors_event_t event; 
  mag.getEvent(&event);
  
  float Pi = 3.14159;

  //Get Mag values
  float rawValueX= event.magnetic.x;
  float rawValueY= event.magnetic.y;
  //Correct mag values against calibration
  float correctedValueX = (((rawValueX - rawLowX) * referenceRange) / rawRangeX) + referenceLow;
  float correctedValueY = (((rawValueY - rawLowY) * referenceRange) / rawRangeY) + referenceLow;
  // Calculate the angle of the vector y,x
  float heading = ( atan2(correctedValueY, correctedValueX) * 180) / Pi;
  
  // Normalize to 0-360
  if (heading < 0)
  {
    heading = 360 + heading;
  }
  Serial.print("Compass Heading: ");
  Serial.println(heading);
  delay(500);
}
