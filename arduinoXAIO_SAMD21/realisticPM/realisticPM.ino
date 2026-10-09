#include <Wire.h>
#include <Adafruit_NeoPixel.h>
#include "SparkFun_Particle_Sensor_SN-GCJA5_Arduino_Library.h"
#include "antirtos.h"

/* ---------------- Hardware ---------------- */
#define PIN 9
#define NUM_LEDS 106
#define DP_LED_INDEX 105
#define MAX_BRIGHTNESS 51
#define MIN_BRIGHTNESS 2
#define ANALOG_PIN A2
#define SENSOR_EN_PIN D8
#define DAC_PIN A0

const int sensEN = SENSOR_EN_PIN;
const int ledCurrentSens = A3;

/* ---------------- Scheduling (in 100 ms ticks) ---------------- */
#define DISPLAY_PERIOD_TICKS  5     // 500 ms
#define SERIAL_PERIOD_TICKS   100   // 10 s

/* ---------------- Task queue ---------------- */
typedef void (*TaskFn)(void);
fQ F1(32);                          // plenty of headroom
volatile uint32_t droppedTasks = 0; // counts tasks lost because queue was full

static inline void pushTask(TaskFn task) {
  if (F1.push(task)) droppedTasks++;   // non-zero return = queue full
}

/* ---------------- Globals ---------------- */
uint16_t ledCurrent = 0;
uint16_t dacValue = 400;
uint8_t brightness = MAX_BRIGHTNESS;

Adafruit_NeoPixel strip(NUM_LEDS, PIN, NEO_GRB + NEO_KHZ800);
SFE_PARTICLE_SENSOR myAirSensor;
bool sensorOK = false;

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
#define FILTER_SIZE 32
uint16_t analogBuffer[FILTER_SIZE];
uint32_t analogSum = 0;
uint8_t analogIndex = 0;
bool bufferFilled = false;

#define PM_FILTER_SIZE 32
float pmBuffer[PM_FILTER_SIZE];
uint8_t pmIndex = 0;
bool pmBufferFilled = false;
float pm2_5 = 0.0f;

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

/* ---------------- ADC setup ---------------- */
void setupADC() {
  analogReadResolution(10);
  analogWriteResolution(10);

  while (ADC->STATUS.bit.SYNCBUSY);
  ADC->REFCTRL.bit.REFSEL = ADC_REFCTRL_REFSEL_INT1V_Val;   // internal 1 V reference
  while (ADC->STATUS.bit.SYNCBUSY);
  ADC->INPUTCTRL.bit.GAIN = ADC_INPUTCTRL_GAIN_1X_Val;
  while (ADC->STATUS.bit.SYNCBUSY);
  analogRead(ledCurrentSens);   // discard first reading
}

/* ---------------- TC4 setup: 100 ms tick ---------------- */
void setupTC4() {
  PM->APBCMASK.reg |= PM_APBCMASK_TC4;

  // GCLK0 (48 MHz) -> TC4/TC5
  GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID(TC4_GCLK_ID) |
                      GCLK_CLKCTRL_GEN_GCLK0 |
                      GCLK_CLKCTRL_CLKEN;
  while (GCLK->STATUS.bit.SYNCBUSY);

  TC4->COUNT16.CTRLA.reg = TC_CTRLA_SWRST;
  while (TC4->COUNT16.STATUS.bit.SYNCBUSY);
  while (TC4->COUNT16.CTRLA.bit.SWRST);

  TC4->COUNT16.CTRLA.reg = TC_CTRLA_MODE_COUNT16 |
                           TC_CTRLA_WAVEGEN_MFRQ |
                           TC_CTRLA_PRESCALER_DIV1024;

  // 48 MHz / 1024 = 46875 Hz. MFRQ period = CC0 + 1 ticks.
  // 4687 + 1 = 4688 ticks = 100.01 ms
  TC4->COUNT16.CC[0].reg = 4687;
  while (TC4->COUNT16.STATUS.bit.SYNCBUSY);

  TC4->COUNT16.INTENSET.reg = TC_INTENSET_MC0;
  NVIC_SetPriority(TC4_IRQn, 2);
  NVIC_EnableIRQ(TC4_IRQn);

  TC4->COUNT16.CTRLA.reg |= TC_CTRLA_ENABLE;
  while (TC4->COUNT16.STATUS.bit.SYNCBUSY);
}

/* ---------------- Setup ---------------- */
void setup() {
  // Power the sensor BEFORE talking to it
  pinMode(sensEN, OUTPUT);
  digitalWrite(sensEN, HIGH);

  pinMode(DAC_PIN, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  Serial.begin(9600);
  unsigned long t0 = millis();
  while (!Serial && (millis() - t0 < 2000)) { ; }

  delay(100);                    // let the sensor boot
  Wire.begin();
  sensorOK = myAirSensor.begin();
  Serial.print("Start. Sensor begin: ");
  Serial.println(sensorOK ? "OK" : "FAILED");

  strip.begin();
  strip.clear();
  strip.show();

  uint8_t idx = 0;
  for (int d = 0; d < 4; d++)
    for (int s = 0; s < 7; s++) {
      segmentStartIndex[d][s] = idx;
      idx += ledsPerSegment[d];
    }

  setupADC();

  ledCheck();                    // runs BEFORE the timer starts

  setupTC4();                    // start 100 ms tick last
}

/* ---------------- Brightness control ---------------- */
void ledBrightnessCtrl(uint8_t target) {
  ledCurrent = analogRead(ledCurrentSens);
  if (ledCurrent < target && dacValue < 1023) dacValue++;
  if (ledCurrent > target && dacValue > 0) dacValue--;
  analogWrite(DAC_PIN, dacValue);
}

void calculateAnalog() {
  analogSum -= analogBuffer[analogIndex];
  analogBuffer[analogIndex] = analogRead(ANALOG_PIN);
  analogSum += analogBuffer[analogIndex];

  analogIndex = (analogIndex + 1) % FILTER_SIZE;
  if (!analogIndex) bufferFilled = true;

  uint16_t avg = bufferFilled ? analogSum / FILTER_SIZE : analogBuffer[0];

  brightness = map(avg, 0, 1023, MAX_BRIGHTNESS, MIN_BRIGHTNESS);
  ledBrightnessCtrl(brightness);
}

/* ---------------- PM handling ---------------- */
void updatePM() {
  pm2_5 = myAirSensor.getPM2_5();
}

void displayPM() {
  pmBuffer[pmIndex++] = pm2_5 * 10.0f;
  if (pmIndex >= PM_FILTER_SIZE) {
    pmIndex = 0;
    pmBufferFilled = true;
  }

  float sum = 0;
  uint8_t count = pmBufferFilled ? PM_FILTER_SIZE : pmIndex;
  for (uint8_t i = 0; i < count; i++) sum += pmBuffer[i];

  int avg = (int)((sum / count) + 0.5f);
  if (avg > 9999) avg = 9999;
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

  uint8_t r = 0, g = 0, b = 0;
  if (c == 'r') r = brightness;
  if (c == 'g') g = brightness;
  if (c == 'b') b = brightness;
  if (c == 'y') r = g = brightness / 2;

  uint32_t col = strip.Color(r, g, b);
  bool leadingZero = true;

  for (int digit = 0; digit < 4; digit++) {
    int num = d[digit];
    bool skip = leadingZero && digit < 2 && num == 0;
    if (!skip) leadingZero = false;

    for (int seg = 0; seg < 7; seg++) {
      bool on = (!skip) && digit_segments[num][seg];
      uint8_t start = segmentStartIndex[digit][seg];
      uint8_t cnt = ledsPerSegment[digit];
      for (int i = 0; i < cnt; i++)
        strip.setPixelColor(start + i, on ? col : 0);
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

  char buf[32];
  int n = snprintf(buf, sizeof(buf), "R%04X%04X%04X%04XM\r\n",
                   v_pm10, v_pm25, v_pm1, checksum);

  // One single write (one USB packet) instead of three separate prints
  if (n > 0) Serial.write((const uint8_t*)buf, n);

  // Debug: uncomment to watch for lost tasks (should stay 0)
  // Serial.print("dropped: "); Serial.println(droppedTasks);
}

/* ---------------- Loop ---------------- */
void loop() {
  F1.pull();
}

/* ---------------- TC4 ISR - fires every 100 ms ---------------- */
void TC4_Handler() {
  static uint8_t tickSerial  = 0;
  static uint8_t tickDisplay = 0;

  if (TC4->COUNT16.INTFLAG.bit.MC0) {
    TC4->COUNT16.INTFLAG.reg = TC_INTFLAG_MC0;

    // Every 10 s - read PM and send to serial (pushed first = runs first)
    if (++tickSerial >= SERIAL_PERIOD_TICKS) {
      tickSerial = 0;
      pushTask(updatePM);
      pushTask(sendPMtoSerial);
    }

    // Every 500 ms - update display
    if (++tickDisplay >= DISPLAY_PERIOD_TICKS) {
      tickDisplay = 0;
      pushTask(displayPM);
    }

    // Every 100 ms - brightness control
    pushTask(calculateAnalog);

    // No CC[0] reload needed in MFRQ mode
  }
}
