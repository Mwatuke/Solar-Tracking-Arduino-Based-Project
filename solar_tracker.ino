/*
  1-axis (east-west) solar tracker with TWO servos driving the SAME axis
  (Arduino Uno / Nano)

  - 2 LDRs: east on A0, west on A1 (each in a divider with a 10k resistor)
  - Panel voltage on A2 through a 10k/10k divider
      Panel +  --> load resistor --> GND   (the load the panel works into)
      Panel +  --> 10k --> A2 --> 10k --> GND   (voltage divider)
      Panel -  --> GND (shared with the Arduino GND)
    The sketch prints voltage, power (P = V^2/R) and accumulated energy (Wh).
  - Servo A on D9, Servo B on D10 (both turn the same shaft)
  - Servos are powered from a SEPARATE 5-6 V supply; grounds joined with the Arduino

  If the two servos face each other (mirror-mounted), set MIRROR_B = true so
  Servo B turns the opposite way to Servo A. Use OFFSET_B to line the two up so
  they do not fight each other (fighting servos draw high current and get hot).
*/

#include <Servo.h>

// ---------- Pins ----------
const int PIN_EAST  = A0;
const int PIN_WEST  = A1;
const int PIN_PANEL = A2;
const int PIN_SERVO_A = 9;
const int PIN_SERVO_B = 10;

// ---------- Settings you can change ----------
const int POS_MIN = 10, POS_MAX = 170, POS_START = 90;  // safe travel range (degrees)

const bool MIRROR_B = false;  // true if Servo B is mounted facing the opposite way to A
const int  OFFSET_B = 0;      // trim for Servo B in degrees (try -5..+5 to stop fighting)

const int DEADBAND = 30;                 // ADC counts (0-1023): bigger = less jitter
const int STEP_DEG = 1;                  // degrees per update
const unsigned long LOOP_MS = 50;        // control period
const unsigned long LOG_MS  = 1000;      // logging period

const int DIR = 1;            // set to -1 if the panel turns AWAY from the light

// ----- Panel measurement -----
const float DIVIDER_RATIO = 2.0;     // 10k + 10k halves the voltage
const float VREF = 5.0;              // Arduino supply (USB is often 4.7-5.0 V)
const float CAL_FACTOR = 1.00;       // calibrate: multimeter reading / Arduino reading
const float LOAD_OHMS = 47.0;        // value of the load resistor across the panel
const int   PANEL_SAMPLES = 32;      // averaging reduces noise

// ---------- Variables ----------
Servo servoA, servoB;
int pos = POS_START;
unsigned long lastLoop = 0, lastLog = 0;
float energyWh = 0.0;                // accumulated electrical energy

int readAvg(int pin) {
  long sum = 0;
  for (int i = 0; i < 8; i++) sum += analogRead(pin);
  return sum / 8;
}

// Panel voltage in volts
float readPanelVolts() {
  long sum = 0;
  for (int i = 0; i < PANEL_SAMPLES; i++) sum += analogRead(PIN_PANEL);
  float adc = sum / (float)PANEL_SAMPLES;
  return adc * (VREF / 1023.0) * DIVIDER_RATIO * CAL_FACTOR;
}

void writeServos(int angle) {
  servoA.write(angle);
  int b = MIRROR_B ? (180 - angle) : angle;
  b = constrain(b + OFFSET_B, 0, 180);
  servoB.write(b);
}

void setup() {
  Serial.begin(9600);
  servoA.attach(PIN_SERVO_A);
  servoB.attach(PIN_SERVO_B);
  writeServos(pos);
  delay(500);
  Serial.println(F("time_s,East,West,Angle_deg,Panel_V,Power_W,Energy_Wh"));
}

void loop() {
  unsigned long now = millis();

  if (now - lastLoop >= LOOP_MS) {
    lastLoop = now;

    int east = readAvg(PIN_EAST);
    int west = readAvg(PIN_WEST);

    // Brighter LDR gives a HIGHER reading in this wiring.
    int diff = west - east;
    if (abs(diff) > DEADBAND) {
      int newPos = pos + DIR * ((diff > 0) ? STEP_DEG : -STEP_DEG);
      newPos = constrain(newPos, POS_MIN, POS_MAX);
      if (newPos != pos) {        // write only when the position changes
        pos = newPos;
        writeServos(pos);
      }
    }
  }

  // CSV line for your experiments (paste from the Serial Monitor into a spreadsheet)
  if (now - lastLog >= LOG_MS) {
    float dt = (now - lastLog) / 1000.0;      // seconds since last log
    lastLog = now;

    float panelV = readPanelVolts();
    float powerW = (panelV * panelV) / LOAD_OHMS;   // P = V^2 / R
    energyWh += powerW * dt / 3600.0;               // integrate power over time

    Serial.print(now / 1000.0, 1); Serial.print(',');
    Serial.print(analogRead(PIN_EAST)); Serial.print(',');
    Serial.print(analogRead(PIN_WEST)); Serial.print(',');
    Serial.print(pos); Serial.print(',');
    Serial.print(panelV, 2); Serial.print(',');
    Serial.print(powerW, 3); Serial.print(',');
    Serial.println(energyWh, 5);
  }
}
