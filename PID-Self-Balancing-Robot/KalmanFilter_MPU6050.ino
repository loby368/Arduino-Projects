#include <Wire.h>
#include <MPU6050.h>
#include <KalmanFilter.h>

MPU6050 mpu;

KalmanFilter kalmanX(0.001, 0.003, 0.03);
KalmanFilter kalmanY(0.001, 0.003, 0.03);

float accPitch = 0;
float accRoll = 0;
float kalPitch = 0;
float kalRoll = 0;

//Pins for Motor Control
int MA1 = 8;    //Motor A Direction
int MA2 = 9;    //Motor A Direction
int EA = 5;     //Motor A Speed
int MB3 = 3;    //Motor B Direction
int MB4 = 4;    //Motor B Direction
int EB = 6;     //Motor B Speed

/*==================================================================Setup=======================================================================*/

void setup() 
{
  Serial.begin(115200);

  // Initialize MPU6050
  while(!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G))
  {
    delay(500);
  }
 
  // Calibrate gyroscope. The calibration must be at rest.
  // If you don't want calibrate, comment this line.
  mpu.calibrateGyro();
}

/*==================================================================Setup=======================================================================*/






/*================================================================Functions=====================================================================*/

// Balance Function
void Balance()
{
  //Declare Variables
  float Speed;
  float Correction_Input;
  char Direction1, Direction2;

  // Read in the current Angle (** NOTE: Ideal Angle is Presumed to be 0 degrees **)
  // PRESUME FORWARD TILT IS POSITIVE, BACKWARDS TILT IS NEGATIVE
  Vector acc = mpu.readNormalizeAccel();
  Vector gyr = mpu.readNormalizeGyro();

  // Calculate Pitch & Roll from accelerometer (deg)
  accPitch = -(atan2(acc.XAxis, sqrt(acc.YAxis*acc.YAxis + acc.ZAxis*acc.ZAxis))*180.0)/M_PI;
  float Current_Angle = kalmanY.update(accPitch, gyr.YAxis);

  //Convert Angle in degrees to a proportional change in speed of motors
  Correction_Input = abs(map(Current_Angle, -90, 90, -12000, 12000));  //This means that a 1 degree error changes the speed value outputed to the motors by 40
  constrain(Correction_Input, 50, 255);

  // Correct for Forward Tilt
  if (Current_Angle != 0)
  {
    Speed = Correction_Input;
  }

  //Correct Forward Tilt
  if ( Current_Angle > 0 )
  {
    Direction1 = LOW;
    Direction2 = HIGH;
  }
  else if ( Current_Angle < 0 )
  {
    Direction1 = HIGH;
    Direction2 = LOW;
  }

  //Output these corrections to the motors
  //Motor A
  digitalWrite(MA1, Direction1);
  digitalWrite(MA2, Direction2);
  analogWrite (EA, Speed);

  //Motor B
  digitalWrite(MB3, Direction1);
  digitalWrite(MB4, Direction2);
  analogWrite (EB, Speed);

  delay (10);
}

/*================================================================Functions=====================================================================*/

void loop()
{
  /*
  Vector acc = mpu.readNormalizeAccel();
  Vector gyr = mpu.readNormalizeGyro();
  // Calculate Pitch & Roll from accelerometer (deg)
  accPitch = -(atan2(acc.XAxis, sqrt(acc.YAxis*acc.YAxis + acc.ZAxis*acc.ZAxis))*180.0)/M_PI;
  // Kalman filter
  kalPitch = kalmanY.update(accPitch, gyr.YAxis);
  Serial.print("    ");
  Serial.print(kalPitch);
  Serial.println();
  */

  Balance();
}
