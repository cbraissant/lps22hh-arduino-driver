#include <Arduino.h>
#include <SPI.h>
#include <LPS22.h>

// SCKL = 13
// SDO (MISO) = 12
// SDI (MOSI) = 11
LPS22 LPS22_a = LPS22();
LPS22 LPS22_b = LPS22();

float P_a, P_b;

void setup() {
  // Initialise the serial monitor
  Serial.begin(9600);
  Serial.println("Programm started");

  // Initialise the SPI
  // Use a global configuration as all the chips are the same.
  // Refactor if another device which use different parameters had to be connected
  SPI.begin();
  SPI.beginTransaction(SPISettings(LPS_SPI_CLOCK_FREQUENCY, MSBFIRST, SPI_MODE3));

  // The ODR is set to "one-shot" by default,
  // and need to be configure to get some readings
  LPS22_a.setCsPin(10);
  LPS22_b.setCsPin(9);
  LPS22_a.setDataRate(LPS_ODR_10_HZ);
  LPS22_b.setDataRate(LPS_ODR_10_HZ);
}

void loop() {
  if (LPS22_a.hasNewPressure()){
    P_a = LPS22_a.getPressure();
  }
  if (LPS22_b.hasNewPressure()){
    P_b = LPS22_b.getPressure();
  }
  Serial.print(P_a);
  Serial.print(",");
  Serial.println(P_b);

}