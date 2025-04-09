//////////////////////
// Driver Libraries //
//////////////////////

#include <Servo.h> // NOTE: This library assumes a 50 Hz period
#include "HX711.h"

//////////////////////////
// I/O port definitions //
//////////////////////////

// Load Cell Amplifier pins
#define LCA_DOUT 3
#define LCA_CLK  2
HX711 loadCell;

// Current Sensor pins
#define CS_PIN A0

// Thruster pins
#define SERVO_PIN 5
Servo thruster;

//////////////////////////////
// Thruster PWM definitions //
//////////////////////////////
const int P_LOW     = 1100;
const int P_NEUTRAL = 1500;
const int P_HIGH    = 1900;

/////////////////////////////////
// PWM Profile type defintions //
/////////////////////////////////

enum SegmentType { Constant, Linear };

typedef struct {
  SegmentType type;
  unsigned long timeframe[2];
  int* values;
} ProfileSegment;

////////////////////////////////
// Current profile definition //
////////////////////////////////

const ProfileSegment PwmProfile[4] = {
  {
    .type = SegmentType::Constant,
    .timeframe = {0, 5000},
    .values = (int[]) {P_NEUTRAL},
  },
  {
    .type = SegmentType::Linear,
    .timeframe = {5000, 10000},
    .values = (int[]) {P_NEUTRAL, P_HIGH},
  },
  {
    .type = SegmentType::Constant,
    .timeframe = {10000, 15000},
    .values = (int[]) {P_HIGH},
  },
  {
    .type = SegmentType::Linear,
    .timeframe = {15000, 20000},
    .values = (int[]) {P_HIGH, P_NEUTRAL},
  },
};

// NOTE: Make sure to double check this after editing the profile above
const unsigned long PROFILE_DURATION_MS = 20000;

//////////////////////////////
// Initialization Constants //
//////////////////////////////

// The amount of time to delay before entering loop(), to allow
// enough time to start the serial monitor PowerShell script
#define SETUP_DELAY 10000 // ms

///////////////////////////////
// Sensor-specific variables //
///////////////////////////////

#define LCA_CALIB_FACTOR -7050.0      // NOTE: This value is obtained using the SparkFun_HX711_Calibration sketch
#define LCA_AVG_SAMPLES 5             // Number of samples to average the reading over
                                      // NOTE: Higher values may lead to buffer overflows
#define CS_SENSITIVITY 100.0 / 500.0  // 100mA per 500mV = 0.2
#define CS_VREF 2500                  // Output voltage with no current: ~ 2500mV or 2.5V
#define CS_AVG_SAMPLES 10             // Number of samples to average the reading over
                                      // NOTE: Higher values may lead to buffer overflows

//////////////////////////////////////
// Sensor-specific helper functions //
//////////////////////////////////////

float fetchCurrentReading() {
  int sensorVal = 0;

  // Read raw analog value
  for (int i = 0; i < CS_AVG_SAMPLES; i++) {
    sensorVal += analogRead(CS_PIN);

    // NOTE: Wait 2 milliseconds per iteration for ADC converter to settle after the last reading
    delay(2);
  }

  // Get averaged sensor value
  sensorVal /= CS_AVG_SAMPLES;

  // On-board ADC is 10-bits, therefore resolution is: 2^10 = 1024 -> 5V / 1024 ~= 4.88mV per bit
  float voltage = 4.88 * sensorVal; // in mV

  // Calculate current (in mA) using calibrated Vref and sensitivity settings
  float current = (voltage - CS_VREF) * CS_SENSITIVITY;

  return current;
}

////////////////////
// Main functions //
////////////////////

// NOTE: Current Sensor just needs to read from A0, so it's pre-initialized
void setup() {
  // NOTE: Supported baud rates are 300, 600, 1200, 2400, 4800, 9600, 14400, 19200, 28800, 31250, 38400, 57600, 115200
  Serial.begin(115200);

  // Setup thruster
  thruster.attach(SERVO_PIN, P_LOW, P_HIGH);

  // Setup Load Cell Amplifier
  loadCell.begin(LCA_DOUT, LCA_CLK);
  loadCell.set_scale(LCA_CALIB_FACTOR);
  loadCell.tare(); // Zero out load sensor assuming there is no applied load

  // Start thruster ESC initialization
  delay(4000);
  
  Serial.println("Initializing ESC: Setting PWM Channel to HIGH...");
  thruster.writeMicroseconds(P_HIGH);
  delay(4000);

  Serial.println("Initializing ESC: Setting PWM Channel to LOW...");
  thruster.writeMicroseconds(P_LOW);
  delay(4000);

  Serial.println("Initializing ESC: Setting PWM Channel to NEUTRAL...");
  thruster.writeMicroseconds(P_NEUTRAL);
  delay(4000);

  // // Delay setup to allow enough time to start the script to read from the serial port ("Read-Arduino-Data.ps1")
  // delay(SETUP_DELAY);

  // Write header of CSV data
  Serial.print("Timestamp (ms),");
  Serial.print("Elapsed Time (us),");
  Serial.print("PWM Signal,");
  Serial.print("Thrust Force,");
  Serial.print("Flowing Current");
  Serial.println();
}


void loop() {
  unsigned long startUs = micros();

  // Get elapsed time (in ms) relative to PWM profile
  unsigned long elapsedProfileMs = (startUs / 1000 - SETUP_DELAY) % PROFILE_DURATION_MS;
  int pwmValue = -1;

  // Select PWM profile based on elapsed MS
  for (ProfileSegment seg : PwmProfile) {
    // Find the current segment to apply on thruster
    if (elapsedProfileMs >= seg.timeframe[0] && elapsedProfileMs < seg.timeframe[1]) {
      if (seg.type == SegmentType::Constant) {
        pwmValue = seg.values[0];
      } else if (seg.type == SegmentType::Linear) {
        // NOTE: Output can have min and max flipped for negative sloping behavior
        // Math formula: (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min
        // Function definition: map(value, fromLow, fromHigh, toLow, toHigh)
        pwmValue = map(elapsedProfileMs, seg.timeframe[0], seg.timeframe[1], seg.values[0], seg.values[1]);
      } else {
        // Safely set thruster to neutral in case an unhandled segment type occurs
        pwmValue = P_NEUTRAL;
      }

      thruster.writeMicroseconds(pwmValue);

      // Don't check any other profile segments by exiting for loop early
      break;
    }
  }

  // Get sensor readings
  float thrustForce = loadCell.get_units(LCA_AVG_SAMPLES);  // This operation takes 100 ms per sample
  float flowingCurrent = fetchCurrentReading();

  // Record how much time elapsed for fetching sensor readings
  unsigned long elapsedUs = micros() - startUs;

  // Print all data in CSV format for chart visualization
  Serial.print(millis());
  Serial.print(",");
  Serial.print(elapsedUs);
  Serial.print(",");
  Serial.print(pwmValue);
  Serial.print(",");
  Serial.print(thrustForce);
  Serial.print(",");
  Serial.print(flowingCurrent);
  Serial.println();
}
