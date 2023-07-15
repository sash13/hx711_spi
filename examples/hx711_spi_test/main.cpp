#include <Arduino.h>
#include <hx711_spi.h>

// HX711 circuit wiring
const int LOADCELL_DOUT_PIN = 19;
const int LOADCELL_SCK_PIN = 23;

HX711_SPI scale;

void setup() {
  Serial.begin(115200);
  scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
}

void loop() {

  if (scale.is_ready()) {
    //long reading = scale.read_average();
    long reading = scale.read_trunc_mean(10, 2);
    long reading2 = scale.read_average();
    long reading3 = scale.read();

    Serial.print("HX711 reading: ");
    Serial.print(reading);
    Serial.print(",");
    Serial.print(reading2);
    Serial.print(",");
    Serial.println(reading3);
  } else {
    Serial.println("HX711 not found.");
  }

  delay(1000);
  
}