#include <Wire.h>
#include <Adafruit_NeoPixel.h>
#include "SparkFun_Particle_Sensor_SN-GCJA5_Arduino_Library.h"
#include "antirtos.h"

/* ---------------- Hardware ---------------- */
#define PIN 9
#define NUM_LEDS 106
#define DP_LED_INDEX 105
#define MAX_BRIGHTNESS 51
#define ANALOG_PIN A2
#define SENSOR_EN_PIN D8

const int dacPin = A0;
const int sensEN = SENSOR_EN_PIN;
const int ledCurrentSens = A3;

/* ---------------- Globals ---------------- */
uint16_t ledCurrent = 0;
uint16_t dacValue = 400;
uint8_t brightness = MAX_BRIGHTNESS;

Adafruit_NeoPixel strip(NUM_LEDS, PIN, NEO_GRB + NEO_KHZ800);
SFE_PARTICLE_SENSOR myAirSensor;
fQ F1(6);

/* ---------------- Forward declarations ---------------- */
void displayNumber(int value, uint8_t c, uint8_t brightness);

/* ---------------- 7-segment map ---------------- */
const uint8_t digit_segments[10][7] = {
  {1,1,1,1,1,1,0},{0,1,1,0,0,0,0},{1,1,0,1,1,0,1},{1,1,1,1,0,0,1},
  {0,1,1,0,0,1,1},{1,0,1,1,0,1,1},{1,0,1,1,1,1,1},{1,1,1,0,0,0,0},
  {1,1,1,1,1,1,1},{1,1,1,1,0,1,1}
};

uint8_t segmentStartIndex[4][7];
uint8_t ledsPerSegment[4] = {4,4,4,3};

/* ---------------- Filters ---------------- */
#define FILTER_SIZE 50
uint16_t analogBuffer[FILTER_SIZE];
uint32_t analogSum = 0;
uint8_t analogIndex = 0;
bool bufferFilled = false;

#define PM_FILTER_SIZE 100
float pmBuffer[PM_FILTER_SIZE];
uint8_t pmIndex = 0;
bool pmBufferFilled = false;
float pm2_5 = 0.0;

/* ---------------- LED test ---------------- */
void ledCheck() {
  for (int i = 0; i < NUM_LEDS; i++) {
    strip.clear();
    strip.setPixelColor(i, strip.Color(255,255,255));
    strip.show();
    delay(50);
  }
  strip.clear();
  strip.show();
}

/* ---------------- Setup ---------------- */
void setup() {
  strip.begin();

  Serial.begin(9600);
  unsigned long t0 = millis();
  while (!Serial && (millis() - t0 < 2000)) { ; }

  analogReadResolution(10);
  analogWriteResolution(10);

  pinMode(dacPin, OUTPUT);
  pinMode(sensEN, OUTPUT);
  digitalWrite(sensEN, HIGH);

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  Wire.begin();
  myAirSensor.begin();

  ledCheck();

  uint8_t idx = 0;
  for (int d=0; d<4; d++)
    for (int s=0; s<7; s++) {
      segmentStartIndex[d][s] = idx;
      idx += ledsPerSegment[d];
    }

  /* -------- TC4 Configuration -------- */
  // Calibrated for XIAO SAMD21 internal oscillator
  // Using DIV256 to stay within 16-bit counter range
  PM->APBCMASK.reg |= PM_APBCMASK_TC4;

  // Select GCLK0 (48 MHz) as source for TC4/TC5
  GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID(TC4_GCLK_ID) |   // TC4 uses GCM_TC4_TC5
                      GCLK_CLKCTRL_GEN_GCLK0 |
                      GCLK_CLKCTRL_CLKEN;
  while (GCLK->STATUS.bit.SYNCBUSY);

  // Reset TC4
  TC4->COUNT16.CTRLA.reg = TC_CTRLA_SWRST;
  while (TC4->COUNT16.STATUS.bit.SYNCBUSY);
  while (TC4->COUNT16.CTRLA.bit.SWRST);

  // Configure 16-bit mode, Match Frequency (MFRQ) waveform, prescaler DIV1024
  TC4->COUNT16.CTRLA.reg = TC_CTRLA_MODE_COUNT16 |
                           TC_CTRLA_WAVEGEN_MFRQ |
                           TC_CTRLA_PRESCALER_DIV1024;

  // Calculate precise value for 100 ms:
  // Timer clock = 48 MHz / 1024 ≈ 46875 Hz → ticks per 100 ms = 4687.5 → use 4688 (error ~0.02%)
  TC4->COUNT16.CC[0].reg = 4688;
  while (TC4->COUNT16.STATUS.bit.SYNCBUSY);

  // Enable MC0 interrupt (match on CC0)
  TC4->COUNT16.INTENSET.reg = TC_INTENSET_MC0;
  NVIC_EnableIRQ(TC4_IRQn);

  TC4->COUNT16.CTRLA.reg |= TC_CTRLA_ENABLE;
  while (TC4->COUNT16.STATUS.bit.SYNCBUSY);

}

/* ---------------- Brightness control ---------------- */
void ledBrightnessCtrl(uint8_t target) {
  ledCurrent = analogRead(ledCurrentSens);
  if (ledCurrent < target && dacValue < 1023) dacValue++;
  if (ledCurrent > target && dacValue > 0) dacValue--;
  analogWrite(dacPin, dacValue);
}

void calculateAnalog() {
  analogSum -= analogBuffer[analogIndex];
  analogBuffer[analogIndex] = analogRead(ANALOG_PIN);
  analogSum += analogBuffer[analogIndex];

  analogIndex = (analogIndex + 1) % FILTER_SIZE;
  if (!analogIndex) bufferFilled = true;

  uint16_t avg = bufferFilled ? analogSum / FILTER_SIZE : analogBuffer[0];
  brightness = map(avg, 0, 1023, MAX_BRIGHTNESS, 0);
  ledBrightnessCtrl(brightness);
}

/* ---------------- PM handling ---------------- */
void updatePM() {
  pm2_5 = myAirSensor.getPM2_5();
}

void displayPM() {
  pmBuffer[pmIndex++] = pm2_5 * 10.0;
  if (pmIndex >= PM_FILTER_SIZE) {
    pmIndex = 0;
    pmBufferFilled = true;
  }

  float sum = 0;
  uint8_t count = pmBufferFilled ? PM_FILTER_SIZE : pmIndex;
  for (uint8_t i=0; i<count; i++) sum += pmBuffer[i];

  int avg = (int)((sum / count) + 0.5);
  char color = (avg > 500) ? 'r' : (avg > 150) ? 'y' : (avg > 50) ? 'g' : 'b';

  displayNumber(avg, color, brightness);
}

/* ---------------- Display number ---------------- */
void displayNumber(int value, uint8_t c, uint8_t brightness) {
  int d[4] = {
    (value / 1000) % 10,
    (value / 100) % 10,
    (value / 10) % 10,
    value % 10
  };

  uint8_t r=0,g=0,b=0;
  if (c=='r') r=brightness;
  if (c=='g') g=brightness;
  if (c=='b') b=brightness;
  if (c=='y') r=g=brightness/2;

  uint32_t col = strip.Color(r,g,b);
  bool leadingZero = true;

  for (int digit=0; digit<4; digit++) {
    int num = d[digit];
    bool skip = leadingZero && digit < 2 && num == 0;
    if (!skip) leadingZero = false;

    for (int seg=0; seg<7; seg++) {
      bool on = (!skip) && digit_segments[num][seg];
      uint8_t start = segmentStartIndex[digit][seg];
      uint8_t cnt = ledsPerSegment[digit];
      for (int i=0; i<cnt; i++)
        strip.setPixelColor(start+i, on ? col : 0);
    }
  }

  strip.setPixelColor(DP_LED_INDEX, col);
  strip.show();
}

/* ---------------- Serial output ---------------- */
void sendPMtoSerial() {
  float pm1  = myAirSensor.getPM1_0();
  float pm25 = myAirSensor.getPM2_5();
  float pm10 = myAirSensor.getPM10();

  uint16_t v_pm10 = (uint16_t)(pm10 * 10.0f + 0.5f);
  uint16_t v_pm25 = (uint16_t)(pm25 * 10.0f + 0.5f);
  uint16_t v_pm1  = (uint16_t)(pm1  * 10.0f + 0.5f);

  uint16_t checksum = v_pm10 + v_pm25 + v_pm1;

  Serial.print("@PM");
  Serial.printf("%04X%04X%04X%04X", v_pm10, v_pm25, v_pm1, checksum);
  Serial.println();
}

/* ---------------- Loop ---------------- */
void loop() {
  F1.pull();
}

/* ---------------- TC4 ISR - fires every 100ms ---------------- */
void TC4_Handler() {
  static uint8_t tick100ms = 0;
  static unsigned long isrCount = 0;

  if (TC4->COUNT16.INTFLAG.bit.MC0) {
    TC4->COUNT16.INTFLAG.reg = TC_INTFLAG_MC0;

    isrCount++;
    tick100ms++;

    // Every 100 ms - brightness control
    F1.push(calculateAnalog);

    // Every 500 ms - update sensor and display
    if ((tick100ms % 5) == 0) {
      F1.push(updatePM);
      F1.push(displayPM);
    }

    // Every 10 seconds - serial output
    if (tick100ms >= 10) {
      tick100ms = 0;
      F1.push(sendPMtoSerial);
    }
// Reload CC[0] for next period
    TC4->COUNT16.CC[0].reg = 4688;
    while (TC4->COUNT16.STATUS.bit.SYNCBUSY);
  }
}