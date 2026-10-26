#include <Wire.h>
#include <Adafruit_NeoPixel.h>
#include "antirtos.h"

/* ---------------- PM sensor selection ----------------
   Uncomment exactly ONE of the following lines.
   - PM_SENSOR_SPS30 : Sensirion SPS30 over I2C (address 0x69, SEL pin -> GND)
                       Built-in driver below, no extra library needed.
   - PM_SENSOR_GCJA5 : Panasonic SN-GCJA5 via SparkFun library
------------------------------------------------------- */
#define PM_SENSOR_SPS30
//#define PM_SENSOR_GCJA5

#if defined(PM_SENSOR_SPS30) && defined(PM_SENSOR_GCJA5)
  #error "Select only one PM sensor"
#endif
#if !defined(PM_SENSOR_SPS30) && !defined(PM_SENSOR_GCJA5)
  #error "Select a PM sensor (PM_SENSOR_SPS30 or PM_SENSOR_GCJA5)"
#endif

#ifdef PM_SENSOR_GCJA5
  #include "SparkFun_Particle_Sensor_SN-GCJA5_Arduino_Library.h"
#endif

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
bool sensorOK = false;

// Latest PM readings (ug/m3), shared by display and serial output
float pm1_0 = 0.0f;
float pm2_5 = 0.0f;
float pm10  = 0.0f;

/* ---------------- Forward declarations ---------------- */
void displayNumber(int value, uint8_t c, uint8_t brightness);

/* =====================================================================
   PM sensor drivers
   Common interface:
     bool pmSensorBegin();                          -> true if sensor responds
     bool pmSensorRead(float &p1, float &p25, float &p10); -> true if new data
   ===================================================================== */

#ifdef PM_SENSOR_SPS30
/* ---------------- Sensirion SPS30 (I2C) ----------------
   Protocol: 16-bit command, data in 16-bit words each followed by CRC-8
   (poly 0x31, init 0xFF). Max I2C clock 100 kHz.
---------------------------------------------------------- */
#define SPS30_ADDR              0x69
#define SPS30_CMD_START_MEAS    0x0010
#define SPS30_CMD_STOP_MEAS     0x0104
#define SPS30_CMD_DATA_READY    0x0202
#define SPS30_CMD_READ_VALUES   0x0300
#define SPS30_CMD_FAN_CLEAN     0x5607
#define SPS30_CMD_SERIAL_NUM    0xD033
#define SPS30_CMD_RESET         0xD304
#define SPS30_CMD_DELAY_MS      20

static uint8_t sps30ErrorCount = 0;
#define SPS30_MAX_ERRORS 3            // re-initialise after this many failed reads

static uint8_t sps30Crc(const uint8_t *data, uint8_t len) {
  uint8_t crc = 0xFF;
  for (uint8_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (uint8_t b = 0; b < 8; b++)
      crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x31) : (uint8_t)(crc << 1);
  }
  return crc;
}

// Send a command with optional argument words (args = 2 bytes per word)
static bool sps30SendCmd(uint16_t cmd, const uint8_t *args = nullptr, uint8_t nWords = 0) {
  Wire.beginTransmission(SPS30_ADDR);
  Wire.write((uint8_t)(cmd >> 8));
  Wire.write((uint8_t)(cmd & 0xFF));
  for (uint8_t i = 0; i < nWords; i++) {
    Wire.write(args[2 * i]);
    Wire.write(args[2 * i + 1]);
    Wire.write(sps30Crc(&args[2 * i], 2));
  }
  return Wire.endTransmission() == 0;
}

// Send a command, then read nWords data words (CRC checked, stripped)
static bool sps30ReadCmd(uint16_t cmd, uint8_t *out, uint8_t nWords) {
  if (!sps30SendCmd(cmd)) return false;
  delay(SPS30_CMD_DELAY_MS);

  uint8_t nBytes = nWords * 3;
  if (Wire.requestFrom((uint8_t)SPS30_ADDR, nBytes) != nBytes) return false;

  for (uint8_t i = 0; i < nWords; i++) {
    uint8_t w[2];
    w[0] = Wire.read();
    w[1] = Wire.read();
    uint8_t crc = Wire.read();
    if (crc != sps30Crc(w, 2)) return false;
    out[2 * i]     = w[0];
    out[2 * i + 1] = w[1];
  }
  return true;
}

static float sps30BytesToFloat(const uint8_t *b) {
  uint32_t u = ((uint32_t)b[0] << 24) | ((uint32_t)b[1] << 16) |
               ((uint32_t)b[2] << 8)  |  (uint32_t)b[3];
  float f;
  memcpy(&f, &u, sizeof(f));
  return f;
}

bool pmSensorBegin() {
  // The MCU may have reset while the sensor kept measuring: stop first
  // (ignore result - the sensor NACKs if it was already idle).
  sps30SendCmd(SPS30_CMD_STOP_MEAS);
  delay(SPS30_CMD_DELAY_MS);

  // Read serial number to confirm communication
  uint8_t sn[32];
  if (!sps30ReadCmd(SPS30_CMD_SERIAL_NUM, sn, 16)) return false;
  sn[31] = 0;
  Serial.print("SPS30 serial: ");
  Serial.println((char*)sn);

  // Start measurement, output format = big-endian IEEE754 float (0x0300)
  const uint8_t fmt[2] = {0x03, 0x00};
  if (!sps30SendCmd(SPS30_CMD_START_MEAS, fmt, 1)) return false;
  delay(SPS30_CMD_DELAY_MS);

  sps30ErrorCount = 0;
  return true;
}

bool pmSensorRead(float &p1, float &p25, float &p10) {
  // Data-ready flag: 1 word, second byte = flag
  uint8_t rdy[2];
  if (!sps30ReadCmd(SPS30_CMD_DATA_READY, rdy, 1)) goto fail;
  if (rdy[1] != 0x01) return false;          // no new data yet, keep old values

  {
    // 10 floats = 20 words = 60 bytes on the bus
    // Order: PM1.0, PM2.5, PM4.0, PM10 mass [ug/m3],
    //        NC0.5, NC1.0, NC2.5, NC4.0, NC10 [#/cm3], typical size [um]
    uint8_t d[40];
    if (!sps30ReadCmd(SPS30_CMD_READ_VALUES, d, 20)) goto fail;

    p1  = sps30BytesToFloat(&d[0]);
    p25 = sps30BytesToFloat(&d[4]);
    // PM4.0 at d[8] is available if needed
    p10 = sps30BytesToFloat(&d[12]);
  }
  sps30ErrorCount = 0;
  return true;

fail:
  if (++sps30ErrorCount >= SPS30_MAX_ERRORS) sensorOK = false;  // triggers re-init
  return false;
}

// Optional: manual fan cleaning (takes ~10 s, readings invalid meanwhile).
// The SPS30 also auto-cleans every 168 h of continuous operation by default.
void pmSensorFanClean() {
  sps30SendCmd(SPS30_CMD_FAN_CLEAN);
}
#endif // PM_SENSOR_SPS30

#ifdef PM_SENSOR_GCJA5
/* ---------------- Panasonic SN-GCJA5 (SparkFun library) ---------------- */
SFE_PARTICLE_SENSOR myAirSensor;

bool pmSensorBegin() {
  return myAirSensor.begin();
}

bool pmSensorRead(float &p1, float &p25, float &p10) {
  p1  = myAirSensor.getPM1_0();
  p25 = myAirSensor.getPM2_5();
  p10 = myAirSensor.getPM10();
  return true;
}
#endif // PM_SENSOR_GCJA5

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
  Wire.setClock(100000);         // SPS30 max 100 kHz (also fine for GCJA5)
  sensorOK = pmSensorBegin();
#ifdef PM_SENSOR_SPS30
  Serial.print("Start. Sensor SPS30 begin: ");
#else
  Serial.print("Start. Sensor SN-GCJA5 begin: ");
#endif
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
  // Recover if the sensor was missing at boot or stopped responding
  if (!sensorOK) {
    sensorOK = pmSensorBegin();
    if (!sensorOK) return;       // keep last values
  }

  float p1, p25, p10;
  if (pmSensorRead(p1, p25, p10)) {
    pm1_0 = p1;
    pm2_5 = p25;
    pm10  = p10;
  }
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
static uint16_t pmToWire(float v) {
  if (v < 0.0f) v = 0.0f;
  float x = v * 10.0f + 0.5f;
  if (x > 65535.0f) x = 65535.0f;
  return (uint16_t)x;
}

void sendPMtoSerial() {
  // Uses values cached by updatePM() - no second sensor read
  uint16_t v_pm10 = pmToWire(pm10);
  uint16_t v_pm25 = pmToWire(pm2_5);
  uint16_t v_pm1  = pmToWire(pm1_0);
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
