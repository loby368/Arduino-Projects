#include <Wire.h>
#include <MPU6050.h>
#include <KalmanFilter.h>

MPU6050 mpu;

KalmanFilter kalmanX(0.001, 0.003, 0.03);
KalmanFilter kalmanY(0.001, 0.003, 0.03);

/*******************/

//PID CONSTANST (Experimental)
float kp = 8;
float ki = 0.5z  5t;
float kd = 0.075;

//TARGET ANGLE (Experimental)
float targetAngle = 0;

/*******************/

//Initialise MPU variables
float accPitch = 0;
float kalPitch = 0;

//Assign Pins for Motor Control
int MA1 = 8;    //Right Motor Direction
int MA2 = 9;    //Right Motor Direction
int EA = 5;     //Right Motor Speed
int MB3 = 3;    //Left Motor Direction
int MB4 = 4;    //Left Motor Direction
int EB = 6;     //Left Motor Spped

//Declare Global Variables
float elapsedTime, time, timePrev;
float PID, error, previousError, Speed;
float pid_p = 0;
float pid_i = 0;
float pid_d = 0;

/*==================================================================Setup=======================================================================*/

void setup()
{
  Serial.begin(115200);

  // Initialize MPU6050
  while (!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G))
  {
    delay(100);
  }

  //OPTIONAL CALIBRATION
  //mpu.calibrateGyro();

  //Start Counting time in milliseconds
  time = millis();
}

/*==================================================================Setup=======================================================================*/


/*================================================================Functions=====================================================================*/

void Forwards()
{
  //Right Motor
  digitalWrite(MA1, LOW);
  digitalWrite(MA2, HIGH);
  analogWrite (EA, Speed);
  //Left Motor
  digitalWrite(MB3, LOW);
  digitalWrite(MB4, HIGH);
  analogWrite (EB, Speed);
}

void Backwards()
{
  //Right Motor
  digitalWrite(MA1, HIGH);
  digitalWrite(MA2, LOW);
  analogWrite (EA, Speed);
  //Left Motor
  digitalWrite(MB3, HIGH);
  digitalWrite(MB4, LOW);
  analogWrite (EB, Speed);
}

void Static()
{
  //Right Motor
  digitalWrite(MA1, LOW);
  digitalWrite(MA2, LOW);
  analogWrite (EA, 0);
  //Left Motor
  digitalWrite(MB3, LOW);
  digitalWrite(MB4, LOW);
  analogWrite (EB, 0);
}

/*================================================================Functions=====================================================================*/

void loop()
{
  
  timePrev = time;
  time = millis();
  elapsedTime = (time - timePrev) / 1000;


  //Read in Data From MPU-6050
  Vector Acceleration = mpu.readNormalizeAccel();
  Vector Gyroscope = mpu.readNormalizeGyro();
  
  //Pitch Acceleration
  accPitch = -(atan2(Acceleration.XAxis, sqrt(Acceleration.YAxis * Acceleration.YAxis + Acceleration.ZAxis * Acceleration.ZAxis)) * 180.0) / M_PI;
  
  //Kalman filter
  kalPitch = kalmanY.update(accPitch, Gyroscope.YAxis);


  //Current Error in Angle
  error = kalPitch - targetAngle;

  //Proportional Error
  pid_p = kp * error;

  //Integral Error
  pid_i += (ki * error);

  //Differential Error
  pid_d = kd * ((error - previousError) / elapsedTime);


  //Total PID Value
  PID = pid_p + pid_i + pid_d;
  
  //Get modulous of PID value for speed of motors
  Speed = abs(PID);
  

  //Update the Error Value
  previousError = error;


  //Calculate Direction to turn motors
  if (kalPitch > 0)
  {Forwards();}
  
  if (kalPitch < 0)
  {Backwards();}
  
  if (kalPitch > 45 || kalPitch < -45)
  {Static();}
 
  
  /*
  //Debugging
  Serial.print(PID);
  Serial.print("     ");
  Serial.print(kalPitch);
  Serial.println();  
  delay(50);
  */

}
