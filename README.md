LOGGER: Una vez enciende el led de la tarjeta, 
en 5 segundos comienza a escribir los datos 
(Gyro3, Acc3, Mag3, Pressure/hPa) en la EEPROM. 
Para la Arduino Nano (1024 bytes) caben 25 lecturas. 
Por defecto lecturas cada 200 ms.

READER: reads all values for gyro or any variable. 
Modify accordingly to export string data.

A4: SDA
A5: SCL
(NANO)