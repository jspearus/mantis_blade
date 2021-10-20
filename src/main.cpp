

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
const int TEM_Sen = A15;
const int BAT_Sen = A12;

// Connect to the RST pin on the Sound Board
#define SFX_RST 41

//################### FUNCTIONDECLAREATIONS ###############
void moveMotor(int val);
void serialEvent();
void serial4Event();

//############## PID Values ###############################
float KP = 3.5; //4
float KI = 1.0; //1
float KD = 0;   //0

int I_S_Offset = 4; //5

int setpoint = 0;

//############################ VARIABLES ########################################
String Data_In = "";
int POSsen = 0;
int intTemp = 0;
float batV = 0;
int dbTemp = 0;
bool ctrlbat_low = false;
float dbatv[4];
float drivePwr = 0.0;
int dbats[4];
unsigned long sysClock;
int updateTime = 5000;
int hudView = 0;

int mode = 0;
bool mode_set = false;

boolean WiiMote = true;

bool isOPen = false;

//PID Variablesd
double Setpoint, Input, Output;

//Specify the links and initial tuning parameters
PID PID1(&Input, &Output, &Setpoint, KP, KI, KD, P_ON_E, DIRECT);
//P_ON_M specifies that Proportional on Measurement be used
//P_ON_E (Proportional on Error) is the default behavior

//####################FUNCTION DECLARATIONS #############################
void internalTemperature();
void getDriveBatData();

Adafruit_Soundboard sfx = Adafruit_Soundboard(&Serial3, NULL, SFX_RST);

BTS7960 MotorController(M_EN, M_L_PWM, M_R_PWM);
Nunchuk nchuk;

void setup()
{
  Serial.begin(115200);  //PC Debug
  Serial3.begin(9600);   ///SoundBoard Port
  Serial4.begin(115200); ///Sensor Glove Port
  Serial4.setTimeout(50);
  Serial5.begin(9600); // XBee port
  Serial5.setTimeout(50);
  Serial6.begin(115200); ///Drive Battery Monitor Port
  Serial6.setTimeout(50);
  delay(3000);
  Serial.println("mantis_blade Initializing...");
  Serial5.println("mantis_blade Initializing...");
  if (!sfx.reset())
  {
    Serial5.println("  Soundboard not found...");
    while (1)
      ;
  }
  Serial5.println(" SFX board found...");
  sfx.println("#01");

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
    intTemp = analogRead(TEM_Sen);
    Serial6.print("dbt#");
    Serial6.print("dbs#");
    Serial6.print("dbv#");
  }

  delay(2500);
  //turn the PID on and set parameters
  PID1.SetMode(AUTOMATIC);
  PID1.SetOutputLimits(-255, 255);
  PID1.SetSampleTime(10);
  Setpoint = 100;
  Serial5.println(" PID ready..");
  Serial5.print(" Mode = ");
  Serial5.println(mode);
  sfx.println("#09");
  Serial5.print("Internal Temp = ");
  internalTemperature();
  Serial5.println(intTemp);
  Serial5.print("CRTL Bat = ");
  getDriveBatData();
  Serial5.println(batV);

  // initialize system clock
  sysClock = millis();
}

void loop()
{
  // update system clock
  if (sysClock + updateTime < millis())
  {
    internalTemperature();
    getDriveBatData();
    // update hudview
    // if (hudView == 2)
    // {
      Serial6.print("dbt#");
      Serial6.print("dbs#");
      Serial6.print("dbv#");
      // todo fix motherboard to hud serial protocol
      // Serial.println(" Drv_Pwr = " + String(drivePwr) + " V");
      // Serial.println(" Drv Bat Temp = " + String(dbTemp) + " C");
      // Serial.println(" Ctrl Bat Temp = " + String(intTemp) + " C");
      // Serial.println(" Ctrl Bat V = " + String(batV) + " V");
      Serial.println("stat," + String(drivePwr) + "," + String(dbTemp) + "," + String(dbats[0]) +
                     "," + String(dbats[1]) + "," + String(dbats[2]) +
                     "," + String(dbats[3]) + "," + String(intTemp) + "," + String(batV) + ",");
    // }
    sysClock = millis(); //Reset SysClock
  }

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
      mode = nchuk.buttonZ();
      //bool crtl = nchuk.buttonC();
      setpoint = nchuk.joyY();
      setpoint = map(setpoint, 0, 255, 0, 500);
      setpoint = constrain(setpoint, 0, 500);
      if (mode == 1 && WiiMote == true)
      {
        Serial5.print("setpoint");
        Serial5.print(" ");
        Serial5.println(setpoint);
      }
    }
  }
  if (WiiMote == false)
  {
    Serial4.print("gfd#");
  }
  int POSread = analogRead(POS_Sen);
  // Serial5.println(POSread); //               POSsen OUTPUT########################
  int POSsen = map(POSread, 220, 970, 0, 500);
  POSsen = constrain(POSsen, 0, 500);
  Input = POSsen;
  setpoint = map(setpoint, 0, 200, 0, 500);
  setpoint = constrain(setpoint, 0, 500);

  if (mode == 1)
  {
    // Dynamic mode
    Setpoint = setpoint;
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
    if (mode_set == false && mode > 0)
    {
      moveMotor(Output);
    }
  }

  else if (mode == 2 && mode_set == false)
  {
    //hold mode
    if (setpoint > 400)
    {
      isOPen = !isOPen;
      Serial5.println(isOPen);
      while (setpoint > 400)
      {
        Serial4.print("gfd#");
        setpoint = map(setpoint, 25, 210, 0, 500);
        setpoint = constrain(setpoint, 0, 500);
      }
    }
    if (isOPen == false)
    {
      Setpoint = 0;
    }

    else if (isOPen == true)
    {
      Setpoint = 500;
    }
    PID1.Compute();
    moveMotor(Output);
    delay(250);
  }
}

void serialEvent()
{ // From HUDpc
  Data_In = Serial.readStringUntil('#');
  if (Data_In == "comm")
  {
    Serial5.println(Data_In);
    Serial.println("MANTIS,");
    Data_In = "";
    sfx.println("#13");
  }
  else if (Data_In == "dash")
  {
    Serial5.println(Data_In);
    Data_In = "";
    sfx.println("#08");
    Serial.println("Dash,");
  }
  else if (Data_In == "config")
  {
    Serial5.println(Data_In);
    Data_In = "";
    sfx.println("#14");
    Serial.println("Config,");
  }
  else if (Data_In == "ctrlt")
  {
    hudView = 2;
    sfx.println("#12");
    Serial6.print("dbs#");
    Serial6.print("dbt#");
    Serial6.print("dbv#");
    // todo fix motherboard to hud serial protocol
    // Serial.println(" Drv_Pwr = " + String(drivePwr) + ",");
    // Serial.println(" Drv Bat Temp = " + String(dbTemp) + ",");
    // Serial.println(" Cell 1 Status = " + String(dbats[0]));
    // Serial.println(" Cell 2 Status = " + String(dbats[1]));
    // Serial.println(" Cell 3 Status = " + String(dbats[2]));
    // Serial.println(" Cell 4 Status = " + String(dbats[3]));
    // Serial.println(" Ctrl Bat Temp = " + String(intTemp) + " C");
    // Serial.println(" Ctrl Bat V = " + String(batV) + " V");
    Serial.println("stat," + String(drivePwr) + ',' + String(dbTemp) + "," + String(dbats[0]) +
                   "," + String(dbats[1]) + ',' + String(dbats[2]) +
                   "," + String(dbats[3]) + "," + String(intTemp) + "," + String(batV) + ",");
    Serial5.println(Data_In);
    Data_In = "";
  }
  else if (Data_In == "exov")
  {
    hudView = 1;
    // Serial.println(" Ctrl Bat Temp = " + String(intTemp));
    Serial5.println(Data_In);
    Data_In = "";
  }
  else if (Data_In == "modeS")
  {
    // Safe Mode from HUD
    Serial5.println(Data_In);
    mode = 0;
    Setpoint = 5;
    PID1.Compute();
    moveMotor(Output);
    Serial.print("mode = ");
    Serial.println(String(mode));
    Data_In = "";
  }
  else if (Data_In == "modes")
  {
    // SYNC mode From HUD
    Serial5.println(Data_In);
    mode = 1;
    Serial.print("mode = ");
    Serial.println(String(mode));
    Data_In = "";
  }
  else if (Data_In == "modeh")
  {
    // HOLD Mode from HUD
    Serial5.println(Data_In);
    mode = 2;
    Serial.print("mode = ");
    Serial.println(String(mode) + ',');
    Data_In = "";
  }
}
void serialEvent6()
{ //Drive Battery Monitor
  Data_In = Serial6.readStringUntil('#');
  if (Data_In == "batMon")
  {
    Serial5.println("Drvie bat found");
    Data_In = "";
  }
  else
  {
    String type = Data_In.substring(0, Data_In.indexOf("@"));
    if (type == "dbt")
    {
      dbTemp = Data_In.substring(Data_In.indexOf("@") + 1, Data_In.indexOf("#")).toFloat();
      // Serial5.println(Data_In);
    }
    else if (type == "dbv")
    {
      dbatv[0] = Data_In.substring(Data_In.indexOf("@") + 1, Data_In.indexOf("-")).toFloat();
      dbatv[1] = Data_In.substring(Data_In.indexOf("-") + 1, Data_In.indexOf(",")).toFloat();
      dbatv[2] = Data_In.substring(Data_In.indexOf(",") + 1, Data_In.indexOf("_")).toFloat();
      dbatv[3] = Data_In.substring(Data_In.indexOf("_") + 1, Data_In.indexOf("#")).toFloat();
      drivePwr = dbatv[0] * 4;
      // Serial5.println(dbatv[0]);
      // Serial5.println(dbatv[1]);
      // Serial5.println(dbatv[2]);
      // Serial5.println(dbatv[3]);
    }
    else if (type == "dbs")
    {
      dbats[0] = Data_In.substring(Data_In.indexOf("@") + 1, Data_In.indexOf("-")).toInt();
      dbats[1] = Data_In.substring(Data_In.indexOf("-") + 1, Data_In.indexOf(",")).toInt();
      dbats[2] = Data_In.substring(Data_In.indexOf(",") + 1, Data_In.indexOf("_")).toInt();
      dbats[3] = Data_In.substring(Data_In.indexOf("_") + 1, Data_In.indexOf("#")).toInt();
      // Serial5.println(dbats[0]);
      // Serial5.println(dbats[1]);
      // Serial5.println(dbats[2]);
      // Serial5.println(dbats[3]);
    }
    else
    {
      Data_In.substring(Data_In.indexOf("@") + 1, Data_In.indexOf("#"));
      Serial.println(Data_In);
    }
    Data_In = "";
  }
}

void serialEvent5()
{ //From XBee
  Data_In = Serial5.readStringUntil('#');
  if (Data_In == "dbt")
  {
    Serial6.print("dbt#");
    Data_In = "";
  }
  else if (Data_In == "dbv")
  {
    Serial6.print("dbv#");
    Data_In = "";
  }
  else if (Data_In == "dbs")
  {
    Serial6.print("dbs#");
    Data_In = "";
  }
  else if (Data_In == "mbo")
  {
    Setpoint = 500;
    PID1.Compute();
    moveMotor(Output);
    Data_In = "";
  }
  else if (Data_In == "mbc")
  {
    Setpoint = 0;
    PID1.Compute();
    moveMotor(Output);
    Data_In = "";
  }
}

void serialEvent4()
{ //from Sensor Glove
  // add it to the inputString:
  Data_In = Serial4.readStringUntil('#');
  // Serial5.println(Data_In); //               SensorGlove OUTPUT ########################
  if (Data_In == "mode")
  {
    if (mode < 2)
    {
      mode++;
    }
    else
    {
      mode = 0;
    }
    Serial5.print(" mode = ");
    Serial5.println(mode);
    Serial.print("mode = ");
    Serial.println(String(mode));
  }
  else if (Data_In == "s")
  {
    mode_set = true;
    Setpoint = 0;
    PID1.Compute();
    moveMotor(Output);
    Serial5.println(" Config...");
    Serial.println("config");
  }
  else if (Data_In == "a")
  {
    mode_set = false;
    Serial5.println(" armed...");
    Serial.println("armed");
  }
  else if (mode_set == false && mode > 0)
  {
    setpoint = Data_In.toInt();
    // Serial5.print("setpoint");
    // Serial5.print(" ");
    // Serial5.println(setpoint);
  }
  Data_In = "";
}

void moveMotor(int val)
{
  if (val > 3)
  {
    MotorController.Enable();
    MotorController.TurnLeft(val);
    //Serial5.println(" Out");
  }
  else if (val < -3)
  {
    val = val * -1;
    MotorController.Enable();
    MotorController.TurnRight(val);
    //Serial5.println(" IN");
  }
  else
  {
    MotorController.Stop();
    MotorController.Disable();
    //Serial5.println(" Stop");
  }
}

void internalTemperature()
{
  int reading = analogRead(TEM_Sen);
  float voltage = reading * 3.3;
  voltage /= 1024.0;
  float temp = (voltage - 0.5) * 100;
  intTemp = int(temp);
}

void getDriveBatData()
{
  float K = 0.003300;
  double cell_const = 1.3800;
  double cellVoltage = analogRead(BAT_Sen) * K;
  cellVoltage *= cell_const;
  batV = cellVoltage;
  if (batV < 3.15 && ctrlbat_low == false)
  {
    sfx.println("#07");
    ctrlbat_low = true;
    Serial.println("ctrlblow");
    Serial5.println("Ctrl Battery Low!!!");
  }
}