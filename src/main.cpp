#include <Arduino.h>
#include <SPI.h>
#include <lps22hh_driver.h>

#define BAUD 256000
#define ARDUINO_CS_PIN 10

LPS22HH_driver sensor;
SPIClass theSpi; 

// Signal     Color       UNO   MEGA
// Vcc        brown    
// GND        white    
// SDO (MISO) yellow      12    50
// SDI (MOSI) orange      11    51
// SCK        blue        13    52
// CS         black

void setup() {
  Serial.begin(BAUD);

  theSpi.begin();
  theSpi.beginTransaction(SPISettings(LPS22HH_SPI_CLOCK_FREQUENCY, MSBFIRST, SPI_MODE0));

  Serial.println("- Output initialized");

  sensor.init(&theSpi, ARDUINO_CS_PIN);

  sensor.reset();

  // The offset pressure (RPDS) is non-volatile and need to be manually reset.
  sensor.setPressureOffset(0);

  // The ODR is set to "one-shot" by default,
  // and need to be configure to get some readings
  sensor.setDataRate(LPS22HH_ODR_200_HZ);

  // Set the Block Data Update (BDU) to 1:
  // output registers not updated until MSB and LSB have been read
  sensor.setBlockDataUpdate(LPS22HH_BDU_SYNCHRONOUS_UPDATE);  
};

float pressure;
char buffer[10];
int previousTime;
int timeDelta = 3;
int newTime;

void loop() {
  newTime = millis();
  if (newTime - previousTime >= timeDelta){
    pressure = sensor.getPressure();
    dtostrf(pressure, 8, 2, buffer);
    previousTime = newTime;
    Serial.println(buffer); 
  }
}
