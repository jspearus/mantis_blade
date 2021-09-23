

#include <Arduino.h>
#include "BTS7960.h"
#include <PID_v1.h>
#include <NintendoExtensionCtrl.h>
#include "Adafruit_Soundboard.h"
#include <EEPROM.h>

//################# PIN DEFINITIONS ########################
const uint8_t M_EN = 4;
const uint8_t M_L_PWM = 2;
const uint8_t M_R_PWM = 3;

const int POS_Sen = A13;

// Connect to the RST pin on the Sound Board
#define SFX_RST 41

//################### FUNCTIONDECLAREATIONS ###############
void moveMotor(int val);

//############## PID Values ###############################
int KP = 4; //4
int KI = 1; //1
int KD = 0; //0

int I_S_Offset = 4; //5

//############################ VARIABLES ########################################
String Data_In = "";
int POSsen = 0;

//PID Variables
double Setpoint, Input, Output;

//Specify the links and initial tuning parameters
PID PID1(&Input, &Output, &Setpoint, KP, KI, KD, P_ON_E, DIRECT);
//P_ON_M specifies that Proportional on Measurement be used
//P_ON_E (Proportional on Error) is the default behavior

Adafruit_Soundboard sfx = Adafruit_Soundboard(&Serial3, NULL, SFX_RST);

BTS7960 MotorController(M_EN, M_L_PWM, M_R_PWM);
Nunchuk nchuk;

boolean WiiMote = true;

boolean zButton = false;
int setpoint = 0;

void setup()
{
  Serial.begin(115200);  //PC Debug
  Serial3.begin(9600);   ///SoundBoard Port
  Serial4.begin(115200); ///Sensor Glove Port
  Serial4.setTimeout(100);
  Serial5.begin(9600); // XBee port
  delay(1000);
  if (!sfx.reset())
  {
    Serial5.println("  Soundboard not found...");
    while (1)
      ;
  }
  Serial5.println(" SFX board found...");

  if (WiiMote == true)
  {
    nchuk.begin();
    delay(1000);
    if (!nchuk.connect())
    {
      Serial5.println(" Sensor glove mode...");
      WiiMote = false;
      delay(1000);
    }
    else
    {
      Serial5.println(" Nunchuk Connected...");
      sfx.println("#10");
    }
  }

  delay(2500);
  //turn the PID on and set parameters
  PID1.SetMode(AUTOMATIC);
  PID1.SetOutputLimits(-255, 255);
  PID1.SetSampleTime(10);
  Setpoint = 100;
  Serial5.println(" PID ready..");
  sfx.println("#02");
}

void loop()
{
  if (WiiMote == true)
  {
    boolean success = nchuk.update(); // Get new data from the controller

    if (!success)
    { // Lost conection with controller
      Serial5.println(" Nunchuk  disconnected!");
      Serial5.println(" Please reset motor Controller!");
      delay(1000);
    }
    else
    {
      zButton = nchuk.buttonZ();
      setpoint = nchuk.joyY();

      // Serial.print(setpoint);
      // Serial.print(" ");
      // Serial.print(Steering);
      // Serial.print(" ");
      // Serial.println(zButton);
    }
  }
  if (WiiMote == false)
  {
    Serial4.print("gfd#");
  }
  int POSsen = analogRead(POS_Sen);
  Serial.println(POSsen);
  POSsen = map(POSsen, 220, 1023, 0, 500);
  POSsen = constrain(POSsen, 0, 500);
  Input = POSsen;
  setpoint = map(setpoint, 0, 300, 0, 500);
  setpoint = constrain(setpoint, 0, 500);
  Setpoint = setpoint;

  Serial.print(Input);
  Serial.print(" ");
  Serial.println(Setpoint);

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

void serialEvent4()
{ //from Sensor Glove
  // add it to the inputString:
  Data_In = Serial4.readStringUntil('#');
  setpoint = Data_In.toInt();
  Data_In = "";
}

void moveMotor(int val)
{
  if (val > 3)
  {
    MotorController.Enable();
    MotorController.TurnLeft(val);
    //Serial.println(" Out");
  }
  else if (val < -3)
  {
    val = val * -1;
    MotorController.Enable();
    MotorController.TurnRight(val);
    //Serial.println(" IN");
  }
  else
  {
    MotorController.Stop();
    MotorController.Disable();
    //Serial.println(" Stop");
  }
}