#include <Arduino.h>
#include "BTS7960.h"
#include <PID_v1.h>
#include "ads.h"
#include <NintendoExtensionCtrl.h>

//################# PIN DEFINITIONS ########################
const uint8_t M_EN = 37;
const uint8_t M_L_PWM = 35;
const uint8_t M_R_PWM = 36;

const int POS_Sen = A19;

BTS7960 MotorController(M_EN, M_L_PWM, M_R_PWM);
Nunchuk nchuk;

//################### FUNCTIONDECLAREATIONS ###############
void moveMotor(int val);

//############## PID Values ###############################
int KP = 4; //4
int KI = 1; //1
int KD = 0; //0

int I_S_Offset = 4; //5

//PID Variables
double Setpoint, Input, Output;

//Specify the links and initial tuning parameters
PID PID1(&Input, &Output, &Setpoint, KP, KI, KD, P_ON_E, DIRECT);
//P_ON_M specifies that Proportional on Measurement be used
//P_ON_E (Proportional on Error) is the default behavior

boolean zButton = false;
int Throttle = 0;
int Steering = 0;

void setup()
{
  Serial.begin(9600); //PC Debug
  delay(2000);
  //turn the PID on and set parameters
  nchuk.begin();
  delay(500);
  Serial.println("WII");
  while (!nchuk.connect())
  {
    Serial.println("Nunchuk not detected!");
    delay(1000);
  }

  PID1.SetMode(AUTOMATIC);
  PID1.SetOutputLimits(-255, 255);
  PID1.SetSampleTime(10);
  Setpoint = 100;
  Serial.println("ready...");
}

void loop()
{

  boolean success = nchuk.update(); // Get new data from the controller

  if (!success)
  { // Lost conection with controller
    Serial1.println("Nunchuk  disconnected!");
    Serial1.println("Please reset motor Controller!");
    delay(1000);
  }
  else
  {
    zButton = nchuk.buttonZ();
    Steering = nchuk.accelY();
    Throttle = nchuk.joyY();

    // Serial.print(Throttle);
    // Serial.print(" ");
    // // Serial.print(Steering);
    // Serial.print(" ");
    // Serial.println(zButton);
  }

  int POSsen = analogRead(POS_Sen);
  POSsen = map(POSsen, 225, 1023, 0, 500);
  Input = POSsen;
  Throttle = map(Throttle, 0, 255, 0, 500);
  Setpoint = Throttle;
  Serial.print(Input);
  Serial.print(" ");
  Serial.print(Setpoint);

  delay(20);
  if (Input >= Setpoint - I_S_Offset &&
      Input <= Setpoint + I_S_Offset)
  {
    Output = 0.0;
  }
  else
  {
    PID1.Compute();
  }
  moveMotor(Output);
}

void moveMotor(int val)
{
  if (val > 3)
  {
    MotorController.Enable();
    MotorController.TurnLeft(val);
    Serial.println(" Out");
  }
  else if (val < -3)
  {
    val = val * -1;
    MotorController.Enable();
    MotorController.TurnRight(val);
    Serial.println(" IN");
  }
  else
  {
    MotorController.Stop();
    MotorController.Disable();
    Serial.println(" Stop");
  }
}
