

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
float KP = 3.5; // 4
float KI = 1.0; // 1
float KD = 0;   // 0

int I_S_Offset = 4;  // 5
int D_S_OFFSET = 70; // 70

int setpoint = 0;

int PosMIN = 220; // Extended
int PosMAX = 780; // Retracted

int SetpointMIN = 15;
int SetpointMAX = 140;

int maxInputRange = 750;

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
String Dbats[4];
unsigned long sysClock;
int updateTime = 5000;
int hudView = 0;

int mode = 0;
bool mode_set = false;
bool quick_mode = false;

boolean WiiMote = true;

bool isOPen = false;

// PID Variablesd
double Setpoint, Input, Output;

// Specify the links and initial tuning parameters
PID PID1(&Input, &Output, &Setpoint, KP, KI, KD, P_ON_E, DIRECT);
// P_ON_M specifies that Proportional on Measurement be used
// P_ON_E (Proportional on Error) is the default behavior

//####################FUNCTION DECLARATIONS #############################
void internalTemperature();
void getDriveBatData();

Adafruit_Soundboard sfx = Adafruit_Soundboard(&Serial3, NULL, SFX_RST);

BTS7960 MotorController(M_EN, M_L_PWM, M_R_PWM);
Nunchuk nchuk;

void setup()
{
  Serial3.begin(9600); /// SoundBoard Port
  Serial3.setTimeout(50);

  Serial4.begin(115200); /// Sensor Glove Port
  Serial4.setTimeout(50);

  Serial5.begin(9600); // XBee port
  Serial5.setTimeout(50);

  Serial6.begin(115200); /// Drive Battery Monitor Port
  Serial6.setTimeout(50);

  Serial7.begin(115200); /// Cyberdeck Port
  Serial7.setTimeout(50);

  delay(3000);
  Serial5.println("mantis_arm Initializing...");
  if (!sfx.reset())
  {
    Serial5.println("  Soundboard not found...");
    while (1)
      ;
  }
  Serial5.println(" Soundboard found...");
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
    Serial5.print(" Min POS:- ");
    Serial5.println(PosMIN);
    Serial5.print("Max POS:- ");
    Serial5.println(PosMAX);
    intTemp = analogRead(TEM_Sen);
    Serial6.print("dbt#");
    Serial6.print("dbs#");
    Serial6.print("dbv#");
  }

  delay(2500);
  // turn the PID on and set parameters
  PID1.SetMode(AUTOMATIC);
  PID1.SetOutputLimits(-255, 255);
  PID1.SetSampleTime(10);
  Setpoint = 100;
  Serial5.println(" PID ready..");
  Serial5.print(" Mode = ");
  Serial5.println(mode);
  sfx.println("#02");
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
  { // update BateSry and Temp data
    internalTemperature();
    getDriveBatData();
    Serial6.print("dbt#");
    Serial6.print("dbs#");
    Serial6.print("dbv#");
    Serial7.println("stat," + String(drivePwr) + "," + String(dbTemp) + "," + String(Dbats[0]) +
                    "," + String(Dbats[1]) + "," + String(Dbats[2]) +
                    "," + String(Dbats[3]) + "," + String(intTemp) + "," + String(batV) + ",");
    sysClock = millis(); // Reset SysClock
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
      // bool crtl = nchuk.buttonC();
      setpoint = nchuk.joyY();
      setpoint = map(setpoint, 0, 255, 0, maxInputRange);
      setpoint = constrain(setpoint, 0, maxInputRange);
      if (mode == 1 && WiiMote == true)
      {
        Serial5.print("setpoint");
        Serial5.print(" ");
        Serial5.println(setpoint);
      }
    }
  }
  else if (WiiMote == false)
  {
    if (mode > 0)
    {
      Serial4.print("gfd#");
    }
  }
  int POSread = analogRead(POS_Sen);
  int POSsen = map(POSread, PosMIN, PosMAX, 0, maxInputRange);
  POSsen = constrain(POSsen, 0, maxInputRange);

  // Serial5.print("Position: ");
  // Serial5.print(POSread);

  Input = POSsen;
  setpoint = map(setpoint, SetpointMIN, SetpointMAX, 0, maxInputRange);
  setpoint = constrain(setpoint, 0, maxInputRange);

  // Serial5.print(" setPoint: ");
  // Serial5.println(setpoint);

  if (mode == 1 && mode_set == false)
  {
    isOPen = false;
    // Dynamic mode
    Setpoint = setpoint;
    if (Input >= Setpoint - I_S_Offset &&
        Input <= Setpoint + I_S_Offset)
    {
      Output = 0.0;
    }
    else
    {
      // Serial5.print(" Input: ");
      // Serial5.println(Input);
      Setpoint = Setpoint + D_S_OFFSET;
      PID1.Compute();
    }
    if (mode_set == false && mode > 0)
    {
      moveMotor(Output);
      delay(20);
    }
  }

  else if (mode == 2 && mode_set == false)
  {
    // hold mode
    if (setpoint > 400)
    {
      isOPen = !isOPen;
      Serial5.println(isOPen);
      while (setpoint > (maxInputRange * .8))
      {
        Serial4.print("gfd#");
        setpoint = map(setpoint, SetpointMIN, SetpointMAX, 0, maxInputRange);
        setpoint = constrain(setpoint, 0, maxInputRange);
      }
    }
    if (isOPen == false)
    {
      Setpoint = 0;
    }

    else if (isOPen == true)
    {
      Setpoint = maxInputRange;
    }
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
    delay(250);
  }
  else if (mode == 3 && mode_set == false)
  {
    isOPen = false;
    // Remote mode
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
  else if (mode == 0 && mode_set == false)
  {
    isOPen = false;
    Setpoint = 0;
    setpoint = 0;
    PID1.Compute();
    moveMotor(Output);
  }
}

void serialEvent7()
{ // From CyberDeck
  Data_In = Serial7.readStringUntil('#');
  if (Data_In == "comm")
  {
    Serial5.println(Data_In);
    Serial7.println("MANTIS,");
    Data_In = "";
    sfx.println("#13");
  }
  else if (Data_In == "dash")
  {
    Serial5.println(Data_In);
    Data_In = "";
    Serial7.println("Dash,");
  }
  else if (Data_In == "config")
  {
    Serial5.println(Data_In);
    Data_In = "";
    Serial7.println("Config,");
  }
  else if (Data_In == "ctrlt")
  {
    hudView = 2;
    Serial6.print("dbs#");
    Serial6.print("dbt#");
    Serial6.print("dbv#");
    Serial7.println("stat," + String(drivePwr) + ',' + String(dbTemp) + "," + String(Dbats[0]) +
                    "," + String(Dbats[1]) + ',' + String(Dbats[2]) +
                    "," + String(Dbats[3]) + "," + String(intTemp) + "," + String(batV) + ",");
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
    Serial7.print("mode = ");
    Serial7.println(String(mode));
    Data_In = "";
  }
  else if (Data_In == "modes")
  {
    // SYNC mode From HUD
    Serial5.println(Data_In);
    mode = 1;
    Serial7.print("mode = ");
    Serial7.println(String(mode));
    Data_In = "";
  }
  else if (Data_In == "qen")
  {
    quick_mode = true;
    Serial4.println("qcm#");
    sfx.println("#03");
    // Quick Click Mode En
    Serial5.println(Data_In);
    Data_In = "";
  }
  else if (Data_In == "qdis")
  {
    quick_mode = false;
    mode = 0;
    Serial4.println("qcmd#");
    // Quick Click Mode Dis
    Serial5.println(Data_In);
    Data_In = "";
  }
  else if (Data_In == "hopen")
  {
    mode = 3;
    Setpoint = maxInputRange;
    Data_In = "";
  }
  else if (Data_In == "hclose")
  {
    mode = 3;
    Setpoint = 0;
    Data_In = "";
  }
  else if (Data_In == "modeh")
  {
    // HOLD Mode from HUD
    Serial5.println(Data_In);
    isOPen = false;
    mode = 2;
    Serial7.print("mode = ");
    Serial7.println(String(mode) + ',');
    Data_In = "";
  }
}
void serialEvent6()
{ // Drive Battery Monitor
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
      for (int i = 0; i < 4; i++)
      {
        if (dbats[i] == 0)
        {
          Dbats[i] = "LOW!!";
        }
        else if (dbats[i] == 1)
        {
          Dbats[i] = "Good";
        }
        else if (dbats[i] == 2)
        {
          Dbats[i] = "Full";
        }
      }
    }
    else
    {
      Data_In.substring(Data_In.indexOf("@") + 1, Data_In.indexOf("#"));
      Serial7.println(Data_In);
    }
    Data_In = "";
  }
}

void serialEvent5()
{ // From XBee
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
  else if (Data_In == "ping")
  {
    Serial5.print("HERE");
    Data_In = "";
  }
  else if (Data_In == "dbs")
  {
    Serial6.print("dbs#");
    Data_In = "";
  }
  else if (Data_In == "mbo")
  {
    Setpoint = maxInputRange;
  }
  else if (Data_In == "mbc")
  {
    Setpoint = 10;
    PID1.Compute();
    moveMotor(Output);
    Data_In = "";
  }
  else if (Data_In == "set")
  {
    Serial5.print("SetPoint: ");
    Serial5.println(Setpoint);
    Serial5.print("setPoint: ");
    Serial5.println(setpoint);
  }
  else
  {
    mode = 3;
    Serial5.print("SetPoint: ");
    Serial5.println(Data_In.toInt());
    Serial5.print("Input: ");
    Serial5.println(Input);
    Setpoint = Data_In.toInt();
    Serial5.print("OutPut: ");
    Serial5.println(Output);
    Data_In = "";
  }
}

void serialEvent4()
{ // from Sensor Glove
  // add it to the inputString:
  Data_In = Serial4.readStringUntil('#');
  // Serial5.println(Data_In); //               SensorGlove OUTPUT ########################
  if (Data_In == "sglove")
  {
    sfx.println("#10");
  }
  else if (Data_In == "mode" && mode_set == true)
  {
    if (mode < 2)
    {
      mode++;
    }
    else
    {
      mode = 0;
    }
    setpoint = 0;
    Serial5.print(" mode = ");
    Serial5.println(mode);
    Serial7.print("mode = ");
    Serial7.println(String(mode));
  }

  else if (Data_In == "s")
  {
    sfx.println("#14");
    mode_set = true;
    isOPen = false;
    Setpoint = 0;
    PID1.Compute();
    moveMotor(Output);
    Serial5.println(" Config...");
    Serial7.println("config");
  }
  else if (Data_In == "c")
  {
    if (quick_mode == true)
    {
      if (mode_set == true)
      {
        mode_set = false;
        Serial7.println("armed");
        isOPen = false;
      }
      else if (mode_set == false)
      {
        mode_set = true;
        Serial7.println("config");
        isOPen = false;
      }
      Serial5.println(" C");
    }
  }
  else if (Data_In == "a")
  {
    if (mode > 0)
    {
      sfx.println("#17");
    }
    else
    {
      sfx.println("#11");
    }
    mode_set = false;
    Serial5.println(" armed...");
    Serial7.println("armed");
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
    // Serial5.println(" Out");
  }
  else if (val < -3)
  {
    val = val * -1;
    MotorController.Enable();
    MotorController.TurnRight(val);
    // Serial5.println(" IN");
  }
  else
  {
    MotorController.Stop();
    MotorController.Disable();
    // Serial5.println(" Stop");
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
    Serial7.println("ctrlblow");
    Serial5.println("Ctrl Battery Low!!!");
  }
}