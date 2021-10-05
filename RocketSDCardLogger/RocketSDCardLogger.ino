/*
 * Arduino NANO Rocket Logger  (SDCARD Version)
 * CS=9, SPI+I2C
 */
#include <Wire.h>
#include <SPI.h>
#include "Adafruit_BMP280.h"
#include "MPU9250_asukiaaa.h"
#include <SD.h>

#define BMP_INTERVAL 100 // ms

File logFile;
MPU9250_asukiaaa mySensor;

Adafruit_BMP280 bmp; // use I2C interface
Adafruit_Sensor *bmp_pressure = bmp.getPressureSensor();
Adafruit_Sensor *bmp_temperature = bmp.getTemperatureSensor();

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
                  Adafruit_BMP280::SAMPLING_X1,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X1,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X8,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_63); /* Standby time. */

  if (!SD.begin(9))
  {
    digitalWrite(13, HIGH);
    return;
  }
}

volatile float variables[11] = { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };

void loop() {
  if (mySensor.accelUpdate() == 0) {
    variables[0] = mySensor.accelX();
    variables[1] = mySensor.accelY();
    variables[2] = mySensor.accelZ();
  }

  if (mySensor.gyroUpdate() == 0) {
    variables[3] = mySensor.gyroX();
    variables[4] = mySensor.gyroY();
    variables[5] = mySensor.gyroZ();
  }

  if (mySensor.magUpdate() == 0) {
    variables[6] = mySensor.magX();
    variables[7] = mySensor.magY();
    variables[8] = mySensor.magZ();
  }
  
  static uint32_t prev_ms = millis();
  if (millis() > prev_ms + BMP_INTERVAL) {
          
      sensors_event_t pressure_event;
      bmp_pressure->getEvent(&pressure_event);

      float pressure = (float) pressure_event.pressure;
      variables[9] = pressure;

      sensors_event_t temp_event;
      bmp_temperature->getEvent(&temp_event);

      float temp = (float) temp_event.temperature;
      variables[10] = temp;

      writeToSD(variables);
            
      prev_ms = millis();
  }
}

// Writing to SD can take up to 200ms
void writeToSD(float dict[11]) {
  // Abrir archivo y escribir valor
  logFile = SD.open("datalog.txt", FILE_WRITE);
      
  if (logFile) { 
      logFile.print((String) millis() + ",accX," + dict[0] + "\n");
      logFile.print((String) millis() + ",accY," + dict[1] + "\n");
      logFile.print((String) millis() + ",accZ," + dict[2] + "\n");
      logFile.print((String) millis() + ",gyroX," + dict[3] + "\n");
      logFile.print((String) millis() + ",gyroY," + dict[4] + "\n");
      logFile.print((String) millis() + ",gyroZ," + dict[5] + "\n");
      logFile.print((String) millis() + ",magX," + dict[6] + "\n");
      logFile.print((String) millis() + ",magY," + dict[7] + "\n");
      logFile.print((String) millis() + ",magZ," + dict[8] + "\n");
      logFile.print((String) millis() + ",prs," + dict[9] + "\n");
      logFile.print((String) millis() + ",tmp," + dict[10] + "\n");
 
      logFile.close();
  } else {
    digitalWrite(13, HIGH);
  }
}
