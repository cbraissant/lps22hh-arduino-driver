#include <Arduino.h>
#include <SPI.h>
#include <LPS22.h>

LPS22 LPS22_a = LPS22(10);

void setup() {
  // Initialise the serial monitor
  Serial.begin(115200);
  Serial.println("Programm started");

  // Initialise the SPI
  // Use a global configuration as all the chips are the same.
  // Refactor if another device which use different parameters had to be connected
  SPI.begin();
  SPI.beginTransaction(SPISettings(LPS_SPI_CLOCK_FREQUENCY, MSBFIRST, SPI_MODE3));

  // The ODR is set to "one-shot" by default,
  // and need to be configure to get some readings
  LPS22_a.setDataRate(LPS_ODR_75_HZ);
  
  // Get the ID of the chip
  Serial.print("Who Am I? ");
  Serial.println(LPS22_a.whoAmI());
}

void loop() {
  if (LPS22_a.hasNewPressure()){
    Serial.print(millis());
    Serial.print(" - ");
    Serial.println(LPS22_a.getPressure()); 
  } else {
  }
}