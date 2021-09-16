

#include <Arduino.h>
#include "BTS7960.h"
#include <PID_v1.h>
#include "ads.h"

//################# PIN DEFINITIONS ########################
const uint8_t M_EN = 0;
const uint8_t M_L_PWM = 0;
const uint8_t M_R_PWM = 0;

#define ADS_RESET_PIN (3)     // Pin number attached to ads reset line.
#define ADS_INTERRUPT_PIN (4) // Pin number attached to the ads data ready line.

BTS7960 MotorController(M_EN, M_L_PWM, M_R_PWM);

//########### FUNCTION DECLARATIONS ############
void ads_data_callback(float *sample);
void deadzone_filter(float *sample);
void signal_filter(float *sample);
void parse_com_port(void);

/* Receives new samples from the ADS library */
void ads_data_callback(float *sample, uint8_t sample_type)
{
  if (sample_type == ADS_SAMPLE)
  {
    // Low pass IIR filter
    signal_filter(sample);

    // Deadzone filter
    deadzone_filter(sample);

    Serial.println(sample[0]);
  }
}

void setup()
{
  // put your setup code here, to run once:
  ads_init_t init;

  init.sps = ADS_100_HZ;                         // Set sample rate to 100 Hzlear
  init.ads_sample_callback = &ads_data_callback; // Provide callback for new data
  init.reset_pin = ADS_RESET_PIN;                // Pin connected to ADS reset line
  init.datardy_pin = ADS_INTERRUPT_PIN;          // Pin connected to ADS data ready interrupt
  init.addr = 0;                                 // Update value if non default I2C address is assinged to sensor
}

void loop()
{
  // put your main code here, to run repeatedly:
}

/* Function parses received characters from the COM port for commands */
void parse_com_port(void)
{
  char key = Serial.read();

  switch (key)
  {
  case '0':
    // Take first calibration point at zero degrees
    ads_calibrate(ADS_CALIBRATE_FIRST, 0);
    break;
  case '9':
    // Take second calibration point at 180 degrees
    ads_calibrate(ADS_CALIBRATE_SECOND, 180);
    break;
  case 'c':
    // Restore factory calibration coefficients
    ads_calibrate(ADS_CALIBRATE_CLEAR, 0);
    break;
  case 'r':
    // Start sampling in interrupt mode
    ads_run(true);
    break;
  case 's':
    // Place ADS in suspend mode
    ads_run(false);
    break;
  case 'f':
    // Set ADS sample rate to 200 Hz (interrupt mode)
    ads_set_sample_rate(ADS_200_HZ);
    break;
  case 'u':
    // Set ADS sample to rate to 10 Hz (interrupt mode)
    ads_set_sample_rate(ADS_10_HZ);
    break;
  case 'n':
    // Set ADS sample rate to 100 Hz (interrupt mode)
    ads_set_sample_rate(ADS_100_HZ);
    break;
  default:
    break;
  }
}

/* 
 *  Second order Infinite impulse response low pass filter. Sample freqency 100 Hz.
 *  Cutoff freqency 20 Hz. 
 */
void signal_filter(float *sample)
{
  static float filter_samples[2][6];

  for (uint8_t i = 0; i < 2; i++)
  {
    filter_samples[i][5] = filter_samples[i][4];
    filter_samples[i][4] = filter_samples[i][3];
    filter_samples[i][3] = (float)sample[i];
    filter_samples[i][2] = filter_samples[i][1];
    filter_samples[i][1] = filter_samples[i][0];

    // 20 Hz cutoff frequency @ 100 Hz Sample Rate
    filter_samples[i][0] = filter_samples[i][1] * (0.36952737735124147f) - 0.19581571265583314f * filter_samples[i][2] +
                           0.20657208382614792f * (filter_samples[i][3] + 2 * filter_samples[i][4] + filter_samples[i][5]);

    sample[i] = filter_samples[i][0];
  }
}

/* 
 *  If the current sample is less that 0.5 degrees different from the previous sample
 *  the function returns the previous sample. Removes jitter from the signal. 
 */
void deadzone_filter(float *sample)
{
  static float prev_sample[2];
  float dead_zone = 0.75f;

  for (uint8_t i = 0; i < 2; i++)
  {
    if (fabs(sample[i] - prev_sample[i]) > dead_zone)
      prev_sample[i] = sample[i];
    else
      sample[i] = prev_sample[i];
  }
}