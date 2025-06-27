#include <Adafruit_NeoPixel.h>

#define PIN            9
#define NUM_LEDS       106
#define DP_LED_INDEX   105
#define MAX_BRIGHTNESS 51     // 20% of 255
#define ANALOG_PIN     A2
const int dacPin = A0;        // DAC output pin (true analog out on SAMD21)
const int ledCurrentSens = A3; 
uint16_t ledCurrent = 0;
uint16_t dacValue = 400;

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

// Display a number on 4-digit 7-segment
void displayNumber(int value, uint8_t c, uint8_t brightness) {
  int d0 = (value / 1000) % 10;
  int d1 = (value / 100)  % 10;
  int d2 = (value / 10)   % 10;
  int d3 = value % 10;

    uint8_t r = 0, g = 0, b = 0;

  switch(c){
    case 'b': b = (255 * brightness) / 255; //blue
              if(b == 0) r = 1;
            break;
    case 'r': r = (255 * brightness) / 255;//red
              if(r == 0) r = 1;
            break;
    case 'g': g = (255 * brightness) / 255;//green
              if(g == 0) g = 1;
            break;
    case 'y': r = (128 * brightness) / 255;//green
              g = (128 * brightness) / 255;//green
              if(g == 0) g = 1;
              if(r == 0) r = 1;
            break;        
  }
   


  uint32_t color = strip.Color(r, g, b);

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

void ledBrightnessCtrlTxt(uint8_t brightness){
  ledCurrent = analogRead(ledCurrentSens);
  if(ledCurrent < brightness) if(dacValue < 1023) dacValue++;
  if(ledCurrent > brightness && ledCurrent > 8) if(dacValue > 0) dacValue --;

  analogWrite(dacPin, dacValue);
}

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
   
    
    displayNumber(ledCurrent, 'y', brightness);
    ledBrightnessCtrlTxt(brightness);

  }
}

