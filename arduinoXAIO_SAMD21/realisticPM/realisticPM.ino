#include <Wire.h>
#include <Adafruit_NeoPixel.h>
#include "SparkFun_Particle_Sensor_SN-GCJA5_Arduino_Library.h"
#include "antirtos.h"

#define PIN 9
#define NUM_LEDS 106
#define DP_LED_INDEX 105
#define MAX_BRIGHTNESS 51  // 20% of 255
#define ANALOG_PIN A2
#define SENSOR_EN_PIN D8

const int dacPin = A0;  // DAC output pin (true analog out on SAMD21)
const int sensEN = SENSOR_EN_PIN;

const int ledCurrentSens = A3;
uint16_t ledCurrent = 0;
uint16_t dacValue = 400;

void TC4_Handler();  // Interrupt handler prototype

Adafruit_NeoPixel strip(NUM_LEDS, PIN, NEO_GRB + NEO_KHZ800);
SFE_PARTICLE_SENSOR myAirSensor;

// 7-segment map
const uint8_t digit_segments[10][7] = {
  { 1, 1, 1, 1, 1, 1, 0 }, { 0, 1, 1, 0, 0, 0, 0 }, { 1, 1, 0, 1, 1, 0, 1 }, { 1, 1, 1, 1, 0, 0, 1 }, { 0, 1, 1, 0, 0, 1, 1 }, { 1, 0, 1, 1, 0, 1, 1 }, { 1, 0, 1, 1, 1, 1, 1 }, { 1, 1, 1, 0, 0, 0, 0 }, { 1, 1, 1, 1, 1, 1, 1 }, { 1, 1, 1, 1, 0, 1, 1 }
};

uint8_t segmentStartIndex[4][7];
uint8_t ledsPerSegment[4] = { 4, 4, 4, 3 };

// Filter setup
#define FILTER_SIZE 50
uint16_t analogBuffer[FILTER_SIZE];
uint32_t analogSum = 0;
uint8_t analogIndex = 0;
bool bufferFilled = false;

// PM2.5 averaging filter
#define PM_FILTER_SIZE 100
float pmBuffer[PM_FILTER_SIZE];
uint8_t pmIndex = 0;
bool pmBufferFilled = false;
float pm2_5 = 0.0;

fQ F1(4);

void ledCheck() {
  for (int i = 0; i < NUM_LEDS; i++) {
    strip.clear();
    strip.setPixelColor(i, strip.Color(255, 255, 255));  // white at max brightness
    strip.show();
    delay(100);  // 0.1 second
  }
  strip.clear();
  strip.show();
}

void setup() {
  strip.begin();


  analogReadResolution(10);   // 0–1023
  analogWriteResolution(10);  // DAC: 0–1023

  pinMode(dacPin, OUTPUT);
  pinMode(sensEN, OUTPUT);

  digitalWrite(sensEN, HIGH);

  Wire.begin();
  myAirSensor.begin();
  ledCheck();

  // Initialize LED segment indexing
  uint8_t idx = 0;
  for (int digit = 0; digit < 4; digit++) {
    for (int seg = 0; seg < 7; seg++) {
      segmentStartIndex[digit][seg] = idx;
      idx += ledsPerSegment[digit];
    }
  }

  // Enable GCLK for TC4 and TC5 (clock generator 0)
  GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID(TC4_GCLK_ID) |  // Generic Clock TC4 and TC5
                      GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0;
  while (GCLK->STATUS.bit.SYNCBUSY)
    ;

  // Reset TC4
  TC4->COUNT16.CTRLA.reg = TC_CTRLA_SWRST;
  while (TC4->COUNT16.STATUS.bit.SYNCBUSY || TC4->COUNT16.CTRLA.bit.SWRST)
    ;

  // Set timer mode 16-bit, prescaler 256
  TC4->COUNT16.CTRLA.reg = TC_CTRLA_MODE_COUNT16 |     // 16-bit mode
                           TC_CTRLA_PRESCALER_DIV256;  // Prescaler 256
  while (TC4->COUNT16.STATUS.bit.SYNCBUSY)
    ;

  // Compare value for 100 ms (0.1 sec): 48MHz / 256 = 187500 Hz -> 0.1s = 18750
  TC4->COUNT16.CC[0].reg = 18750;
  while (TC4->COUNT16.STATUS.bit.SYNCBUSY)
    ;

  // Enable interrupt on compare match 0
  TC4->COUNT16.INTENSET.reg = TC_INTENSET_MC0;
  NVIC_EnableIRQ(TC4_IRQn);
  NVIC_SetPriority(TC4_IRQn, 0);

  TC4->COUNT16.CTRLA.reg |= TC_CTRLA_ENABLE;
  while (TC4->COUNT16.STATUS.bit.SYNCBUSY)
    ;
}

void displayNumber(int value, uint8_t c, uint8_t brightness) {
  int d0 = (value / 1000) % 10;
  int d1 = (value / 100) % 10;
  int d2 = (value / 10) % 10;
  int d3 = value % 10;

  uint8_t r = 0, g = 0, b = 0;
  switch (c) {
    case 'b':
      b = brightness;
      if (b == 0) r = 1;
      break;
    case 'r':
      r = brightness;
      if (r == 0) r = 1;
      break;
    case 'g':
      g = brightness;
      if (g == 0) g = 1;
      break;
    case 'y':
      r = g = brightness / 2;
      if (r == 0) r = g = 1;
      break;
  }

  uint32_t color = strip.Color(r, g, b);
  bool leadingZero = true;

  for (int digit = 0; digit < 4; digit++) {
    int number = (digit == 0) ? d0 : (digit == 1) ? d1
                                   : (digit == 2) ? d2
                                                  : d3;

    bool skip = (leadingZero && digit < 2 && number == 0);
    if (!skip) leadingZero = false;

    for (int seg = 0; seg < 7; seg++) {
      bool on = (!skip) && digit_segments[number][seg];
      uint8_t start = segmentStartIndex[digit][seg];
      uint8_t count = ledsPerSegment[digit];
      for (int i = 0; i < count; i++) {
        strip.setPixelColor(start + i, on ? color : 0);
      }
    }
  }
  strip.setPixelColor(DP_LED_INDEX, color);
  strip.show();
}

void ledBrightnessCtrlTxt(uint8_t brightness) {
  ledCurrent = analogRead(ledCurrentSens);
  if (ledCurrent < brightness)
    if (dacValue < 1023) dacValue++;
  if (ledCurrent > brightness && ledCurrent > 8)
    if (dacValue > 0) dacValue--;
  analogWrite(dacPin, dacValue);
}

uint8_t brightness = 0;

void calculateAnalog() {
  analogSum -= analogBuffer[analogIndex];
  analogBuffer[analogIndex] = analogRead(ANALOG_PIN);
  analogSum += analogBuffer[analogIndex];
  analogIndex = (analogIndex + 1) % FILTER_SIZE;
  if (analogIndex == 0) bufferFilled = true;
  uint16_t avgAnalog = bufferFilled ? analogSum / FILTER_SIZE : analogBuffer[0];
  brightness = map(avgAnalog, 0, 1023, MAX_BRIGHTNESS, 0);
  ledBrightnessCtrlTxt(brightness);
}
void updatePM() {
  pm2_5 = myAirSensor.getPM2_5();
}

void displayPM() {
  pmBuffer[pmIndex] = pm2_5 * 10.0;
  pmIndex = (pmIndex + 1) % PM_FILTER_SIZE;
  if (pmIndex == 0) pmBufferFilled = true;

  float pmSum = 0;
  uint8_t count = pmBufferFilled ? PM_FILTER_SIZE : pmIndex;
  for (uint8_t i = 0; i < count; i++) pmSum += pmBuffer[i];
  float avgPM = pmSum / count;

  displayNumber((int)avgPM, 'y', brightness);
}


void loop() {
  F1.pull();
}

void TC4_Handler() {
  static int counter = 0;
  if (TC4->COUNT16.INTFLAG.bit.MC0) {
    TC4->COUNT16.INTFLAG.reg = TC_INTFLAG_MC0;
    F1.push(calculateAnalog);
    F1.push(displayPM);
    if (!(counter % 2)) F1.push(updatePM);
  }
}
