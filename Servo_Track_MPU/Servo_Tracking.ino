#include "MPU9250.h"
#include "eeprom_utils.h"

#include <Servo.h>
Servo pitchservo;
Servo rollservo;

MPU9250 mpu;
int r = 0;
int p = 0;
int y = 0;

/*==================================================================Setup=======================================================================*/
void setup()
{
  pitchservo.attach(7);
  rollservo.attach(6);
  Serial.begin(250000);

  Wire.begin();

  delay(1000);
  mpu.setup(Wire);

  //Call Calibration Function
  //Calibration();

  //Load from eeprom
  loadCalibration();

  mpu.printCalibration();

}
/*==================================================================Setup=======================================================================*/






/*================================================================Functions=====================================================================*/

void Calibration()
{
  //Calibration
  mpu.calibrateAccelGyro();
  mpu.calibrateMag();

  //Save to EEPROM
  saveCalibration();
}

/*================================================================Functions=====================================================================*/

void loop()
{

  mpu.update();

  r = map(mpu.getRoll(), -179, 179, 360, -180);

  p = map(mpu.getPitch(), -90, 90, -150, 330);

  y = map(mpu.getYaw(), -179, 179, 360, -180 );

  Serial.print(r);
  Serial.print("  |  ");
  Serial.print(p);
  Serial.print("  |  ");
  Serial.print(y);
  Serial.print("\n");

  pitchservo.write(r);
  rollservo.write(y);

}
