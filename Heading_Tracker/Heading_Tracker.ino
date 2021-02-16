
/*================================================== INITIALISATION ==========================================*/

// Declare Mode as a global variable
// LEFT TURNS  = Mode 1
// RIGHT TURNS = Mode 2
int Mode;

//#include <LCD16x2.h>            //Allows us to use the LCD at the top
#include <Wire.h>
#include <NineAxesMotion.h>     //Allows us to use the 9 Axis Sensor
#include <EnableInterrupt.h>

//LCD16x2 lcd;
NineAxesMotion mySensor;

int intVal1 = 0, intVal2 = 0;
float floatVal1 = 0.0, floatVal2 = 0.0;
int E1 = 5;                     //Port for M1 Speed Control
int E2 = 6;                     //Port for M2 Speed Control
int M1 = 4;                     //Port for M1 Direction Control
int M2 = 7;                     //Port for M2 Direction Control

//Initialise variables as Global Variables so we can read their values in multiple functions
float Turn_Count = 0;
float Positive_x, Positive_y, Negative_x, Negative_y;
int Backup_Strategy = 0;
volatile int distance = 0, Axle_Count = 0;      // Allows them to be used in the Interrupt
volatile int CycleCount = 0, MatchLength = 240; // 240 seconds is 4 minutes






/*================================================== INITILISATION ===========================================*/





/*====================================================== SETUP ===============================================*/
void setup()
{
  int b; //Variable to store value of white buttons

  //Peripheral Initialization
  I2C.begin(); //Initialize I2C communication to let the library communicate with the sensor.
  Wire.begin();

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  pinMode(2, INPUT_PULLUP);
  enableInterrupt(2, Distance_Driven_Function, RISING); //Triggers the Axle_Count funtion when the limit switch is pressed
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  //Sensor Initialization
  mySensor.initSensor();                            //The I2C Address can be changed here inside this function in the library
  mySensor.setOperationMode(OPERATION_MODE_NDOF);   //NDOF = 9 Degrees of Freedom Sensor (Other operatin modes are available)
  mySensor.setUpdateMode(AUTO);                     //The default is AUTO

  pinMode(E1, OUTPUT);                              //These pins control the motor H-bridge circuitry, so we need to set them up as outputs
  pinMode(E2, OUTPUT);
  pinMode(M1, OUTPUT);
  pinMode(M2, OUTPUT);

//  lcd.lcdSetBlacklight(200);
//  lcd.lcdClear();
//  lcd.lcdGoToXY(1, 1);
//  lcd.lcdWrite("   Go!   |Backup");
//  lcd.lcdGoToXY(1, 2);
//  lcd.lcdWrite("---------|    ||");

//  b = lcd.readButtons();         //Sample the state of the large white buttons
//  while (b == 15)                //A value of 15 will only be seen if no buttons are pressed
//  {
//    b = lcd.readButtons();       //Sample the state of the large white buttons
//  }

  //If the backup strategy button is pressed
  if (b == 7)
  {
    Backup_Strategy = 1;
  }

  arm_and_start();               // Set-up Arming and Start process
}
/*====================================================== SETUP ================================================*/





/***************************************************** FUNCTIONS **********************************************/

//ISR - Distance Driven Function
//NOTE: Because our switch bounces on average 12 times every press, the ISR is triggered 12 times but becahuse 'distance' only increases by 0.73cm every trigger,
// it takes 10 presses to increase the distance by 7.3cm ie. 1/3 a wheel rotation.
float Distance_Driven_Function()
{
  Axle_Count++;
  distance = (0.1) * Axle_Count;   //De-Bouncing. On average, the switch bounces 12 times, so dividing it by 10 gives us an accurate interger value for 1 count.
  distance = distance * (7.3);     //(One-Third of the Circumference of wheel (cm)) * (Number of One-Third Revolutions of wheel)
}






//Set up Heading Framework
void Heading_Setup_Function()
{
  if ( Mode == 1 ) //Left Turns
  {
    Positive_y = 0;    // We start pointing to Positive-y => calculate other headings from this
    Positive_x = 90;
    Negative_y = 180;
    Negative_x = 270;
  }
  if ( Mode == 2 ) //Right Turns
  {
    Positive_y = 180;  // We start pointing to Negative-y => calculate other headings from this
    Positive_x = 270;
    Negative_y = 0;
    Negative_x = 90;
  }
}






//Turn 90 Degrees Right Function
void Turn_RIGHT_Function()
{
  float Start_Heading = mySensor.readEulerHeading();
  float Current_Heading, Target_Heading;

  ///////////////////////// WITHIN 90 DEGREES OF 360/0 MARK /////////////////////////
  if (Start_Heading + 90 >= 360)                               //Checking if the turn will be affected by the 360/0 barrier
  {
    Current_Heading = (mySensor.readEulerHeading() - 360);     //Makes the angle a minus so we can calculate what the Target will Actually be given as

    Target_Heading = (Current_Heading + 90);                   //The robot has passed 360 and started counting from 0, This gives us what the new target will be

    while (Target_Heading >= Current_Heading)
    {
      //TURN CLOCKWISE
      analogWrite (E1, 200);      //Set M1 speed
      digitalWrite(M1, HIGH);     //Set M1 direction
      analogWrite (E2, 200);      //Set M2 speed
      digitalWrite(M2, LOW);      //Set M2 direction

      if (Current_Heading >= -90 && Current_Heading < 0)       //If current heading is before boundary, convert it to minus so it is mathematically below the target
      {
        Current_Heading = (mySensor.readEulerHeading()) - 360; //Makes the angle a minus so it will be less than the Target
      }                                                        //NOTE: Magnitude is preserved
      else
      {
        Current_Heading = mySensor.readEulerHeading();         //If current heading is past the boundary, leave it unchanged
      }
    }
  }
  /////////////////////////NOT WITHIN 90 DEGREES OF 360/0 MARK /////////////////////////
  else
  {
    Current_Heading = mySensor.readEulerHeading();
    Target_Heading = Current_Heading + 90;

    while (Target_Heading > Current_Heading)
    {
      analogWrite (E1, 200);      //Set M1 speed
      digitalWrite(M1, HIGH);     //Set M1 direction
      analogWrite (E2, 200);      //Set M2 speed
      digitalWrite(M2, LOW);      //Set M2 direction

      Current_Heading = mySensor.readEulerHeading();
    }
  }
  analogWrite (E1, 20);           //Set speed very low to cancel out any momentum the robot has
  digitalWrite(M1, LOW);          //Set to Opposite direction to try and stop the robot from over-spinning
  analogWrite (E2, 20);
  digitalWrite(M2, HIGH);
}






//Turn 90 Degrees Left Function
void Turn_LEFT_Function()
{
  float Start_Heading = mySensor.readEulerHeading();
  float Current_Heading, Target_Heading;

  ///////////////////////// WITHIN 90 DEGREES OF 360/0 MARK /////////////////////////
  if (Start_Heading - 90 <= 0)                                 //Checking if the turn will be affected by the 360/0 barrier
  {
    Current_Heading = (mySensor.readEulerHeading() + 360);     //Makes the angle a minus so we can calculate what the Target will Actually be given as

    Target_Heading = (Current_Heading - 90);                   //The robot has passed 360 and started counting from 0, This gives us what the new target will be

    while (Target_Heading <= Current_Heading)
    {
      //TURN COUNTER-CLOCKWISE
      analogWrite (E1, 200);      //Set M1 speed
      digitalWrite(M1, LOW);      //Set M1 direction
      analogWrite (E2, 200);      //Set M2 speed
      digitalWrite(M2, HIGH);     //Set M2 direction

      if (Current_Heading <= 450 && Current_Heading > 0)       //If current heading is before boundary, convert it to minus so it is mathematically below the target
      {
        Current_Heading = (mySensor.readEulerHeading()) + 360; //Makes the angle a minus so it will be less than the Target
      }                                                        //NOTE: Magnitude is preserved
      else
      {
        Current_Heading = mySensor.readEulerHeading();         //If current heading is past the boundary, leave it unchanged
      }
    }
  }
  /////////////////////////NOT WITHIN 90 DEGREES OF 360/0 MARK /////////////////////////
  else
  {
    Current_Heading = mySensor.readEulerHeading();
    Target_Heading = Current_Heading - 90;

    while (Target_Heading < Current_Heading)
    {
      analogWrite (E1, 200);      //Set M1 speed
      digitalWrite(M1, LOW);      //Set M1 direction
      analogWrite (E2, 200);      //Set M2 speed
      digitalWrite(M2, HIGH);     //Set M2 direction

      Current_Heading = mySensor.readEulerHeading();
    }
  }
  analogWrite (E1, 20);           //Set speed very low to cancel out any momentum the robot has
  digitalWrite(M1, LOW);          //Set to Opposite direction to try and stop the robot from over-spinning
  analogWrite (E2, 20);
  digitalWrite(M2, HIGH);
}






//Turn for 90 Degrees Function
void Execute_Turn()
{
  if (Mode == 1)
  {
    Turn_LEFT_Function();
  }
  else if (Mode == 2)
  {
    Turn_RIGHT_Function();
  }

  Turn_Count++;   //Write the number of turns into a global variable so we can read it in other functions
  distance = 0;   //Reset the distance to Zero
  Axle_Count = 0; //Reset the axle-count to zero
}






// Follow Heading while Driving Function
void Drive_Foward()
{
  //Declare Variables
  float Left_Speed, Right_Speed;
  int Error;

  // Read in the current heading
  float Current_Direction = mySensor.readEulerHeading();

  //Calculate Error based on which direction it should be pointing

  //Mode 1
  if ( (Turn_Count == 0 || Turn_Count == 4) && Mode == 1 )
  {
    // Calculate Error from target heading
    Error = Current_Direction - Positive_y;
  }
  else if (Turn_Count == 2 && Mode == 1 )
  {
    // Calculate Error from target heading
    Error = Current_Direction - Negative_y;
  }
  else if (Turn_Count == 3 && Mode == 1 )
  {
    // Calculate Error from target heading
    Error = Current_Direction - Positive_x;
  }
  else if (Turn_Count == 1 && Mode == 1 )
  {
    // Calculate Error from target heading
    Error = Current_Direction - Negative_x;
  }

  //Mode 2
  if ( (Turn_Count == 0 || Turn_Count == 4) && Mode == 2 )
  {
    // Calculate Error from target heading
    Error = Current_Direction - Negative_y;
  }
  else if (Turn_Count == 2 && Mode == 2 )
  {
    // Calculate Error from target heading
    Error = Current_Direction - Positive_y;
  }
  else if (Turn_Count == 1 && Mode == 2 )
  {
    // Calculate Error from target heading
    Error = Current_Direction - Negative_x;
  }
  else if (Turn_Count == 3 && Mode == 2 )
  {
    // Calculate Error from target heading
    Error = Current_Direction - Positive_x;
  }

  //Ensure Error is within 180 either side of target heading
  if (Error > 180)
  {
    Error = -360 + Error;
  }
  else if (Error < -180)
  {
    Error = 360 + Error;
  }

  //Convert Error in degrees to a proportional change in speed
  Error = map(Error, -180, 180, -3600, 3600);//This means that a 1 degree error changes the speed value outputed to the motors by 20
  //This scale was decided by trial and error

  // Correct for Right deviation
  if (Error > 0)
  {
    // Keep the Right wheel spinning at the same speed and Slow down the Left wheel
    Left_Speed = 240 - Error; //Right wheel has to overcome more resistance of the cogs => Left wheel drives slightly slower
    Right_Speed = 255;
  }

  //Correct for Left deviation
  else
  {
    // Keep the Left wheel spinning at the same speed and Slow down the Right wheel
    Left_Speed = 240;          //Right wheel has to overcome more resistance of the cogs => Left wheel drives slightly slower
    Right_Speed = 255 + Error; //(Error is a minus)
  }

  //Output these new speeds to the motors
  analogWrite (E1, Right_Speed);
  digitalWrite(M1, LOW);
  analogWrite (E2, Left_Speed);
  digitalWrite(M2, LOW);
}






//Drive Foward For Distance Function
void Drive_Foward_For(float Required_Distance)
{
  delay(100);            //Allows the robot to settle after a turn

  while ( distance <= Required_Distance )
  {
    Drive_Foward();
  }

  analogWrite (E1, 20);  //Set speed very low in the opposite direction to cancel out any momentum the robot has
  digitalWrite(M1, HIGH);
  analogWrite (E2, 20);
  digitalWrite(M2, HIGH);

  delay(100);            //Allow the robot to settle before turning again

  Axle_Count = 0;        //Reset axle and distance to zero after driving
  distance = 0;
}






//Arming and Start Function
void arm_and_start()
{
  int b = 0;
  int buttons = 0;
  int pointinsecond;
  long countdown_starttime, countdown_length = 10000;

  float timeleft;

  Wire.begin();

  //Start-up Menu
  //lcd.lcdClear();
  //lcd.lcdSetBlacklight(200);
  //lcd.lcdGoToXY(1, 1);
  //lcd.lcdWrite("Lt  Rt");        // Lt: Left Turns = Mode 1, Rt: Right Turns = Mode 2
  //lcd.lcdGoToXY(1, 2);
  //lcd.lcdWrite("|   |");

//  b = lcd.readButtons();         //Sample the state of the large white buttons
//  do
//  {
//    b = lcd.readButtons();       //Sample the state of the large white buttons
//  }
//  while (b != 14 && b != 13);

  //Left button activates Mode 1, Right button activates Mode 2
  if (b == 14)
  {
    Mode = 1;
  }
  else if (b == 13)
  {
    Mode = 2;
  }

  //lcd.lcdClear();
  //lcd.lcdSetBlacklight(200);
  //lcd.lcdGoToXY(1, 1);
  //lcd.lcdWrite("Press outer keys");

//  buttons = lcd.readButtons();   //Sample the state of the large white buttons
//  do
//  {
//    buttons = lcd.readButtons(); //Sample the state of the large white buttons
//  }
//  while (buttons != 6);          // A value of 6 will only be seen if the leftmost and rightmost buttons are pressed
//
//  // Only break to here once buttons is equal to 61
//  lcd.lcdClear();
//  lcd.lcdGoToXY(2, 1);
//  lcd.lcdWrite("Release to arm");
//
//  while (buttons == 6)           //Stay in this loop while buttons are held down
//  {
//    buttons = lcd.readButtons(); //Sample the state of the large white buttons
//  }
//
//  // Once we get here the countdown sequence needs to start
//  countdown_starttime = millis();
//
//  lcd.lcdClear();

  while (millis() < countdown_starttime + countdown_length)
  {
    timeleft = ( (countdown_starttime + countdown_length) - millis() ) / 1000.0;
//
//    lcd.lcdGoToXY(7, 2);
//    lcd.lcdWrite(timeleft, 2);

    pointinsecond = ((countdown_starttime + countdown_length) - millis() ) % 1000;

//    lcd.lcdSetBlacklight(abs(pointinsecond - 500) / 2);
  }

//  lcd.lcdClear();
//  lcd.lcdSetBlacklight(200);
//  lcd.lcdGoToXY(4, 2);
//  lcd.lcdWrite("Go Team 2!");

  //Now set up an interupt to trigger after 4 minutes to shut down the robot

  enableInterrupt(11, CheckCycle, RISING); //Attach an interrupt to pin 11
  tone(11, 31); //Output a 31 Hz square wave to pin 11 to trigger this interrupt
}






// 4 minute timer Function
void CheckCycle(void)
{
  CycleCount++;  //Check how many times we've been here

  if (CycleCount == 31 * MatchLength)
  {
    //turn off motors
    analogWrite (E1, 0); //Turn off M1
    analogWrite (E2, 0); //Turn off M2

    while (1) //loop forever to shut down Arduino
    {
    }
  }
}

/***************************************************** FUNCTIONS **********************************************/





/*==================================================== Main Loop ==============================================*/

void loop()
{

  //Call the heading set-up function so we know what lines to follow
  Heading_Setup_Function();

  //Back-up Strategy just collects one Red ball - "Point & Shoot"
  if (Backup_Strategy == 1)
  {
    Drive_Foward_For(320);
    Execute_Turn();
    Execute_Turn();
    Drive_Foward_For(320);

    delay(1000000);//Since we are only doing a quich drive, we can wait in the box until the game is over
  }

  //Perform driving commands to execute strategy

  // Start Position -> Edge of court
  Drive_Foward_For(66);
  Execute_Turn();

  //First Red Ball
  Drive_Foward_For(285);
  Execute_Turn();

  //Second and Third Red Balls
  Drive_Foward_For(560);
  Execute_Turn();

  //Yellow Ball
  Drive_Foward_For(285);
  Execute_Turn();

  //Home Straight
  Drive_Foward_For(560);

  delay(1000000); //Since we are only doing a quich drive, we can wait in the box until the game is over

}
/*====================================================== Main Loop ================================================*/
