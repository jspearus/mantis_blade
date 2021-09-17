#include <Arduino.h>
#include "BTS7960.h"
#include <PID_v1.h>
#include "ads.h"

//################# PIN DEFINITIONS ########################
const uint8_t M_EN = 37;
const uint8_t M_L_PWM = 35;
const uint8_t M_R_PWM = 36;

const int POS_Sen = A20;

BTS7960 MotorController(M_EN, M_L_PWM, M_R_PWM);

//############## PID Values ###############################
int KP = 4; //4
int KI = 1; //1
int KD = 0; //0

//PID Variables
double Setpoint, Input, Output;

//Specify the links and initial tuning parameters
PID PID1(&Input, &Output, &Setpoint, KP, KI, KD, P_ON_E, DIRECT);
//P_ON_M specifies that Proportional on Measurement be used
//P_ON_E (Proportional on Error) is the default behavior

void setup()
{
    Serial.begin(9600); //PC Debug

    //turn the PID on and set parameters

    PID1.SetMode(AUTOMATIC);
    PID1.SetOutputLimits(-100, 100);
    PID1.SetSampleTime(10);
    Setpoint = 100;
    Serial.println("ready...");
}

void loop()
{
    int POSsen = analogRead(POS_Sen);
    Serial.println(POSsen);
}
