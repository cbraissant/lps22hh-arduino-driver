#include <Arduino.h>
#include <SPI.h>
#include <LPS22.h>

#define DEBUG false

#if DEBUG
#define debug(x) Serial.print(x)
#define debugln(x) Serial.println(x)
#else
#define debug(x)
#define debugln(x)
#endif

#define BAUD 74880
#define ARDUINO_PIN_1ST_CS 2
#define ARDUINO_PIN_DEBUG_OUTPUT 43


const int nbr_sensors = 11;
LPS22 sensors[nbr_sensors];
float pressure[nbr_sensors];

// Signal     Color       UNO   MEGA
// Vcc        brown    
// GND        white    
// SDO (MISO) yellow      12    50
// SDI (MOSI) orange      11    51
// SCK        blue        13    52
// CS         black

void arduino_output_high() {
  for (int i=2; i<=13; i++){
    pinMode(i, OUTPUT);
    digitalWrite(i,HIGH);
  }
}


void setup() {
  Serial.begin(BAUD);
  debugln();
  debugln();
  debugln("--- Setup started ---");
  
  // Initialise the SPI
  // Use a global configuration as all the chips are the same.
  // Refactor if another device which use different parameters had to be connected
  SPI.begin();
  SPI.beginTransaction(SPISettings(LPS_SPI_CLOCK_FREQUENCY, MSBFIRST, SPI_MODE0));
  debugln("- SPI Initialized");

  pinMode(ARDUINO_PIN_DEBUG_OUTPUT, OUTPUT); // DEBUG
  debugln("- Debug output Initialized");

  arduino_output_high();
  debugln("- Outputs Initialized");


  for(int i=0; i<nbr_sensors; i++){
    debug("- Sensor ");
    debug(i);
    debug(": ");

    sensors[i].setCsPin(i+ARDUINO_PIN_1ST_CS);
    debug("Output");

    sensors[i].swreset();
    debug(" - Reset");

    if (sensors[i]._isWorking == false) {
      debugln(" - SENSOR NOT WORKING");
      continue;
    }

    // The ODR is set to "one-shot" by default,
    // and need to be configure to get some readings
    sensors[i].setDataRate(LPS_ODR_200_HZ);
    debug(" - ODR");

    // Set the Block Data Update (BDU) to 1:
    // output registers not updated until MSB and LSB have been read
    sensors[i].writeSingleBit(LPS_CTRL_REG1, 1, 1);

    debug(" - Who Am I: ");
    debugln(sensors[i].whoAmI());
  }
  
  debugln();
  debugln("--- Setup finished ---");
  debugln();

}
unsigned long time = 0;

void updateSerial() {
  for (int i=0; i<nbr_sensors; i++) {
    Serial.print(pressure[i]);
    if (i < nbr_sensors - 1){
      Serial.print(",");
    }
  }
  // Serial.print(",");
  // Serial.print(millis()-time);
  // time = millis();
  Serial.println();
}

bool needUpdating = false;

void loop() {
  for(int i=0; i<nbr_sensors; i++){
    // Read the pressure of each sensor to update the array...
    if (sensors[i]._isWorking) {
      if(sensors[i].hasNewPressure()){
        pressure[i]=sensors[i].getPressure();
        needUpdating = true;
      }
    }
  }
  if (needUpdating) {
    updateSerial();
    needUpdating = false;
  }
}
