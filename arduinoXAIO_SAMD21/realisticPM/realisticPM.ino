#include <Adafruit_NeoPixel.h>

#define PIN            9
#define NUM_LEDS       106
#define DP_LED_INDEX   105
#define MAX_BRIGHTNESS 51     // 20% of 255
#define ANALOG_PIN     A2
const int dacPin = A0;        // DAC output pin (true analog out on SAMD21)

Adafruit_NeoPixel strip(NUM_LEDS, PIN, NEO_GRB + NEO_KHZ800);

// 7-segment map
const uint8_t digit_segments[10][7] = {
  {1,1,1,1,1,1,0}, {0,1,1,0,0,0,0}, {1,1,0,1,1,0,1}, {1,1,1,1,0,0,1},
  {0,1,1,0,0,1,1}, {1,0,1,1,0,1,1}, {1,0,1,1,1,1,1}, {1,1,1,0,0,0,0},
  {1,1,1,1,1,1,1}, {1,1,1,1,0,1,1}
};

uint8_t segmentStartIndex[4][7];
uint8_t ledsPerSegment[4] = {4, 4, 4, 3};

// Filter setup
#define FILTER_SIZE 100
uint16_t analogBuffer[FILTER_SIZE];
uint32_t analogSum = 0;
uint8_t analogIndex = 0;
bool bufferFilled = false;

void setup() {
  strip.begin();
  strip.clear();
  strip.show();
  analogReadResolution(10);   // 0–1023
  analogWriteResolution(10);  // DAC: 0–1023

  pinMode(dacPin, OUTPUT);

  // Initialize LED segment indexing
  uint8_t idx = 0;
  for (int digit = 0; digit < 4; digit++) {
    for (int seg = 0; seg < 7; seg++) {
      segmentStartIndex[digit][seg] = idx;
      idx += ledsPerSegment[digit];
    }
  }
}

// Color wheel with brightness scaling
uint32_t colorWheel(uint16_t pos, uint8_t brightness) {
  pos = pos % 1536;
  uint8_t r = 0, g = 0, b = 0;

  if (pos < 256)         { r = 255;         g = pos;         b = 0; }
  else if (pos < 512)    { r = 511 - pos;   g = 255;         b = 0; }
  else if (pos < 768)    { r = 0;           g = 255;         b = pos - 512; }
  else if (pos < 1024)   { r = 0;           g = 1023 - pos;  b = 255; }
  else if (pos < 1280)   { r = pos - 1024;  g = 0;           b = 255; }
  else                   { r = 255;         g = 0;           b = 1535 - pos; }

  return strip.Color((r * brightness) / 255, (g * brightness) / 255, (b * brightness) / 255);
}

// Display a number on 4-digit 7-segment
void displayNumber(int value, uint16_t hue, uint8_t brightness) {
  int d0 = (value / 1000) % 10;
  int d1 = (value / 100)  % 10;
  int d2 = (value / 10)   % 10;
  int d3 = value % 10;

  uint32_t color = colorWheel(hue, brightness);

  for (int digit = 0; digit < 4; digit++) {
    int number = (digit == 0) ? d0 : (digit == 1) ? d1 : (digit == 2) ? d2 : d3;
    for (int seg = 0; seg < 7; seg++) {
      bool on = digit_segments[number][seg];
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

// Timer and color
unsigned long lastUpdate = 0;
int tenths = 0;
uint16_t hue = 0;

void loop() {
  unsigned long now = millis();
  if (now - lastUpdate >= 100) {
    lastUpdate = now;

    // Moving average filter
    analogSum -= analogBuffer[analogIndex];
    analogBuffer[analogIndex] = analogRead(ANALOG_PIN);
    analogSum += analogBuffer[analogIndex];
    analogIndex = (analogIndex + 1) % FILTER_SIZE;
    if (analogIndex == 0) bufferFilled = true;

    uint16_t avgAnalog = bufferFilled ? analogSum / FILTER_SIZE : analogBuffer[0];

    // Inverted brightness & DAC output
    uint8_t brightness = map(avgAnalog, 0, 1023, MAX_BRIGHTNESS, 0);
    uint16_t dacValue  = map(avgAnalog, 0, 1023, 1023, 0); // full 0–1023 range for DAC

    displayNumber(tenths, hue, brightness);
    analogWrite(dacPin, dacValue);

    tenths++;
    if (tenths >= 10000) tenths = 0;
    hue = (hue + 7) % 1536;
  }
}
