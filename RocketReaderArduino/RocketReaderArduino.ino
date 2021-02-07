/*
 * EEPROM Read
 */

#include <EEPROM.h>

struct LoggerObject { // 10 fields of float
  float acc[3]; // X,Y,Z
//  float gyro[3];
//  float mag[3];
//  float pressure;
};

// start reading from the first byte (address 0) of the EEPROM
int volatile eeAddress = 0;

void setup() {
  // initialize serial and wait for port to open:
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }

  Serial.println(EEPROM.length());

  if(EEPROM.read(0) == 0x00 && EEPROM.read(1) != 0x00){
      Serial.println("First byte is empty. Ok");
  }
  // Empty byte first to indicate data begin
  eeAddress += 1;
}

void loop() {
  // read a byte from the current address of the EEPROM
  LoggerObject data;
  EEPROM.get(eeAddress, data);

  Serial.print("Acc: ");
  Serial.print("\t");
//  Serial.print("["+(String)data.acc[0]+","+(String)data.acc[1]+","+(String)data.acc[2]+","+(String)data.gyro[0]+","+(String)data.gyro[1]+","+(String)data.gyro[2]+","+(String)data.mag[0]+","+(String)data.mag[1]+","+(String)data.mag[2]+","+(String)data.pressure+"]");
  Serial.print("["+(String)data.acc[0]+","+(String)data.acc[1]+","+(String)data.acc[2]+"]");

  Serial.println();

  eeAddress += sizeof(data);
  if (eeAddress >= EEPROM.length()) {
    eeAddress = 1;
  }

  delay(500);
}
