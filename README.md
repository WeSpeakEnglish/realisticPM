<p align="center">
  <img src="https://raw.githubusercontent.com/WeSpeakEnglish/realisticPM/main/realisticpm.jpg" alt="realisticPM" />
</p>
<p align="center">
  <a href="https://www.youtube.com/watch?v=CBKU3_4Pjrw">
    <img src="https://img.youtube.com/vi/CBKU3_4Pjrw/maxresdefault.jpg"
         width="600"
         alt="Watch on YouTube">
  </a>
</p>


# RealisticPM
**RealisticPM** is an on-wall air quality monitor, powered via USB-C connector and based on [Seeeduino XIAO SAMD21](https://wiki.seeedstudio.com/Seeeduino-XIAO/)

## Features

- Big color display
- Standard USB-C connector
- Low power consumption (2 W max)
- High-quality LEDs
- COM port data output (virtual COM port)
- Pin-out for developing purposes
- Open-source code
- Automatic brightness control
- Fused via resettable fuse
- Automatic color control via addressing LEDs
- PM2.5 value display step as low as 0.1 ug/m3
- Sizes of 364x164x18 mm, ideal for work environment / sport rooms / clean rooms
- Panasonic SN-GCJA5 or Sensirion SPS30 PM sensor

## Supported PM sensors

| Sensor | Interface | I2C address | Driver |
|---|---|---|---|
| Sensirion SPS30 | I2C (100 kHz max) | 0x69 | Built into the sketch, no extra library |
| Panasonic SN-GCJA5 | I2C | 0x33 | [SparkFun SN-GCJA5 Arduino Library](https://github.com/sparkfun/SparkFun_Particle_Sensor_SN-GCJA5_Arduino_Library) |

Both sensors are powered at 5 V and switched by the `SENSOR_EN` pin (D8).

### Selecting the sensor

Open `realisticPM.ino` and uncomment exactly one line at the top:

```cpp
#define PM_SENSOR_SPS30
//#define PM_SENSOR_GCJA5
```

The display, brightness control and serial output work the same with either sensor.

### Connecting the SPS30

| SPS30 pin | Function | Connect to |
|---|---|---|
| 1 | VDD | 5 V (switched by `SENSOR_EN`) |
| 2 | SDA | XIAO SDA (D4) |
| 3 | SCL | XIAO SCL (D5) |
| 4 | SEL | GND (selects I2C mode) |
| 5 | GND | GND |

- Pull SDA and SCL up to **3.3 V** (e.g. 10 kΩ). The SAMD21 pins are not 5 V tolerant.
- The SPS30 cleans its fan automatically every 168 h of continuous operation. `pmSensorFanClean()` in the sketch starts a manual cleaning (about 10 s; readings are invalid meanwhile).
- If the sensor is missing at start-up or stops responding, the firmware re-initialises it automatically and keeps showing the last good value.

## Building

1. Install the Seeeduino XIAO (SAMD) board package in the Arduino IDE.
2. Install the libraries:
   - Adafruit NeoPixel
   - SparkFun SN-GCJA5 library (only when `PM_SENSOR_GCJA5` is selected)
3. Keep `antirtos.h` in the sketch folder.
4. Select the sensor (see above), then build and upload.

## Serial output

The monitor appears as a virtual COM port. Every 10 s it sends one line:

```
R<PM10><PM2.5><PM1.0><CHECKSUM>M\r\n
```

- Each field is 4 hex digits, in units of 0.1 ug/m3 (e.g. `0019` = 2.5 ug/m3).
- `CHECKSUM` is the 16-bit sum of the three values.

Example: `R00280019000E004FM` means PM10 = 4.0, PM2.5 = 2.5, PM1.0 = 1.4 ug/m3.

On start-up the firmware also prints which sensor is selected and whether it was found (for the SPS30, its serial number too).

## Display colors

The PM2.5 value (moving average of 32 samples) is shown with color by level:

| PM2.5 (ug/m3) | Color |
|---|---|
| ≤ 5.0 | Blue |
| 5.1 – 15.0 | Green |
| 15.1 – 50.0 | Yellow |
| > 50.0 | Red |
