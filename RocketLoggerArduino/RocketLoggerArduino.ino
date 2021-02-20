
/*
 * Arduino NANO Rocket Logger  (SDCARD Version)
 * CS=9, SPI+I2C
 */
#include <Wire.h>
#include <SPI.h>
#include "Adafruit_BMP280.h"
#include "MPU9250_asukiaaa.h"
#include <SD.h>
#include <ArduinoQueue.h>

#define BMP_INTERVAL 21 // ms
#define QUEUE_INTERVAL 200 //ms

File logFile;
MPU9250_asukiaaa mySensor;

Adafruit_BMP280 bmp; // use I2C interface
Adafruit_Sensor *bmp_pressure = bmp.getPressureSensor();

struct BufferObject {
    char keyword[2]; // Can be: aX,aY,aZ,gX,gY,gZ,mX,mY,mZ,p
    float value;
};

ArduinoQueue<BufferObject> bufferQueue(100, 1024);

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
                  Adafruit_BMP280::FILTER_X2,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_1); /* Standby time. */

  if (!SD.begin(9))
  {
    digitalWrite(13, HIGH);
    return;
  }
}

void loop() {
  if (mySensor.accelUpdate() == 0) {
    BufferObject accx = { "ax", mySensor.accelX()};
    bufferQueue.enqueue(accx);
    BufferObject accy = { "ay", mySensor.accelY()};
    bufferQueue.enqueue(accy);
    BufferObject accz = { "az", mySensor.accelZ()};
    bufferQueue.enqueue(accz);
  }

  if (mySensor.gyroUpdate() == 0) {
    BufferObject gyrx = { "gx", mySensor.gyroX()};
    bufferQueue.enqueue(gyrx);
    BufferObject gyry = { "gy", mySensor.gyroY()};
    bufferQueue.enqueue(gyry);
    BufferObject gyrz = { "gz", mySensor.gyroZ()};
    bufferQueue.enqueue(gyrz);
  }

  if (mySensor.magUpdate() == 0) {
    BufferObject magx = { "magX", mySensor.magX()};
    bufferQueue.enqueue(magx);
    BufferObject magy = { "magY", mySensor.magY()};
    bufferQueue.enqueue(magy);
    BufferObject magz = { "magZ", mySensor.magZ()};
    bufferQueue.enqueue(magz);
  }
  
  static uint32_t prev_ms = millis();
  if (millis() > prev_ms + BMP_INTERVAL) {
          
      sensors_event_t pressure_event;
      bmp_pressure->getEvent(&pressure_event);

      BufferObject pressure = { "pr", pressure_event.pressure};
      bufferQueue.enqueue(pressure);
            
      prev_ms = millis();
  }

  static uint32_t prev_queue_ms = millis();
  if (millis() > prev_queue_ms + QUEUE_INTERVAL) {
    // Abrir archivo y escribir valor
    logFile = SD.open("datalog.txt", FILE_WRITE);

    if (logFile) { 
      //TODO: write all Queue data into 
      /*
      logFile.println(millis()); // ms
      logFile.print(",");
      logFile.print(keyword);
      logFile.print(",");
      logFile.print(value);
       */
      
      logFile.close();
    } else {
      digitalWrite(13, HIGH);
    }
            
    prev_queue_ms = millis();
  }
}
