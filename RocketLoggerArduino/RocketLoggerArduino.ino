
/*
 * Arduino NANO Rocket Logger  (SDCARD Version)
 * CS=9, SPI+I2C
 */
#include <Wire.h>
#include <SPI.h>
#include "Adafruit_BMP280.h"
#include "MPU9250_asukiaaa.h"
#include <SD.h>

#define INTERVAL 100 // ms

struct LoggerObject { // 10 fields of float
  float acc[3]; // X,Y,Z
  float gyro[3];
  float mag[3];
  float pressure;
};

File logFile;
MPU9250_asukiaaa mySensor;
float aX, aY, aZ, aSqrt, gX, gY, gZ, mDirection, mX, mY, mZ;

Adafruit_BMP280 bmp; // use I2C interface
Adafruit_Sensor *bmp_pressure = bmp.getPressureSensor();

void setup() {
  Wire.begin();
  delay(2000);

  pinMode(9, OUTPUT);
  pinMode(13, OUTPUT);

  if (!bmp.begin()) {
    digitalWrite(13, HIGH);
  }

  mySensor.beginAccel();
  mySensor.beginGyro();
  mySensor.beginMag();

  /* Default settings from datasheet. */
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */

  if (!SD.begin(9))
  {
    digitalWrite(13, HIGH);
    return;
  }
}

void loop() {
  if (mySensor.accelUpdate() == 0 && mySensor.gyroUpdate() == 0 & mySensor.magUpdate() == 0) {
        static uint32_t prev_ms = millis();
        if (millis() > prev_ms + INTERVAL) {
          
            sensors_event_t pressure_event;
            bmp_pressure->getEvent(&pressure_event);

            LoggerObject data = {
              {mySensor.accelX(), mySensor.accelY(), mySensor.accelZ()},
              {mySensor.gyroX(), mySensor.gyroY(), mySensor.gyroZ()},
              {mySensor.magX(), mySensor.magY(), mySensor.magZ()},
              (float) pressure_event.pressure,
            };

            // Abrir archivo y escribir valor
            logFile = SD.open("datalog.txt", FILE_WRITE);
            
            if (logFile) { 
                  logFile.print("ms:");
                  logFile.print(millis());
                  logFile.println(data.acc[0]);
                  logFile.println(data.acc[1]);
                  logFile.println(data.acc[2]);
                  logFile.println(data.gyro[0]);
                  logFile.println(data.gyro[1]);
                  logFile.println(data.gyro[2]);
                  logFile.println(data.mag[0]);
                  logFile.println(data.mag[1]);
                  logFile.println(data.mag[2]);
                  logFile.println(data.pressure);
                  
                  logFile.close();
            } else {
              digitalWrite(13, HIGH);
            }
            
            prev_ms = millis();
        }
    }
}
