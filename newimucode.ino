#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

/* This driver uses the Adafruit unified sensor library (Adafruit_Sensor),
   which provides a common 'type' for sensor data and some helper functions.

   To use this driver you will also need to download the Adafruit_Sensor
   library and include it in your libraries folder.

   You should also assign a unique ID to this sensor for use with
   the Adafruit Sensor API so that you can identify this particular
   sensor in any data logs, etc.  To assign a unique ID, simply
   provide an appropriate value in the constructor below (12345
   is used by default in this example).

   Connections
   ===========
   Connect SCL to analog 5
   Connect SDA to analog 4
   Connect VDD to 3-5V DC
   Connect GROUND to common ground

   History
   =======
   2015/MAR/03  - First release (KTOWN)
   2015/AUG/27  - Added calibration and system status helpers
*/

/* Set the delay between fresh samples */
#define BNO055_SAMPLERATE_DELAY_MS (100)

Adafruit_BNO055 bno = Adafruit_BNO055(55,0x29);

bool calibrated = false;

char buf[256];

///**************************************************************************/
/*
    Arduino setup function (automatically called at startup)
*/
/**************************************************************************/
void setup(void)
{
  Serial.begin(9600);

  /* Initialise the sensor */
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }

  Serial.println("Sensor Initiated"); Serial.println("");
  
  delay(1000);

  /* Display some basic information on this sensor */
//  displaySensorDetails();

  /* Optional: Display current status */
//  displaySensorStatus();

  bno.setExtCrystalUse(true);
}

/**************************************************************************/
/*
    Arduino loop function, called once 'setup' is complete (your own code
    should go here)
*/
/**************************************************************************/
void loop(void)
{
  /* Get a new sensor event */
  sensors_event_t event;                                                   //Creates a sensors event called event
  float x,y,z;                                                             //Creates floats for the x, y, and z axis

  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);

  uint8_t system, gyro, accel, mag;                                        //Create local variables gyro, accel, mag
  system = gyro = accel = mag = 0;
  if(calibrated == false){            
    bno.getCalibration(&system, &gyro, &accel, &mag);                    //Read the calibration values from the IMU
    bno.getEvent(&event);                           
    sprintf(buf, "Status: %d %d %d %d", system, gyro, accel, mag);

    Serial.println("Calibrating...");
    Serial.print(system, DEC);
    Serial.print(",");
    Serial.print(gyro, DEC);
    Serial.print(",");
    Serial.print(accel,DEC);
    Serial.print(",");
    Serial.println(mag, DEC);
//    if(bno.isFullyCalibrated() == true){                                 //Will return true if all calibration values are 3
    if(system == 3){                                 //Will return true if all calibration values are 3
            calibrated = true;                                                  //Sets "calibrated" to true, preventing the calibration loop from executing again
      }
    }
  else{
    delay(500);
    x = euler.x();                                                      //Creates Euler Vectors for the x, y, and z axis
    y = euler.y();
    z = euler.z();
      
    float declination = 8.12; // Longmont, Colorado
    float sata = x - 180 - declination;
    sprintf(buf, "SATA: %f", sata);

    float satz = z - 90;
    if(satz < -180)
      {
        satz = satz + 360;
      }
    sprintf(buf, "SATZ: %f", satz);
    
    Serial.println();                                                   //Prints the angles out over the serial port for x, y, and z axis
    Serial.println(sata);
    Serial.println(satz);
  }  
}
