#include <Arduino.h>
#include <hx711_spi.h>

// HX711 circuit wiring
const int LOADCELL_DOUT_PIN = 19;
const int LOADCELL_SCK_PIN = 23;

HX711_SPI scale;
char buffer [100];
char idx = 0;

void setup() {
  Serial.begin(115200);
  scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
}

void loop() {
  uint64_t _lastRead, time_read, time_trunc_mean, time_trunc_mean_slide, time_average;

  if (scale.is_ready()) {
    //long reading = scale.read_average();
    uint64_t _lastRead = micros();
    long read_trunc_mean = scale.read_trunc_mean(10, 2);
    time_trunc_mean = micros() - _lastRead;

    _lastRead = micros();
    long read = scale.read();
    time_read = micros() - _lastRead;

    _lastRead = micros();
    long read_average = scale.read_average();
    time_average = micros() - _lastRead;

    _lastRead = micros();
    long read_trunc_mean_slide = scale.read_trunc_mean(10, 2, 1);
    time_trunc_mean_slide = micros() - _lastRead;

    sprintf(buffer, "Reading #%d", idx);
    Serial.println(buffer);

    sprintf(buffer, "read()\t average()\t trunc_mean()\t trunc_slide()");
    Serial.println(buffer);

    sprintf(buffer, "%d\t %d\t %d\t %d", read, read_average, read_trunc_mean, read_trunc_mean_slide);
    Serial.println(buffer);
    sprintf(buffer, "%d\t %d\t %d\t %d", time_read, time_average, time_trunc_mean, time_trunc_mean_slide);
    Serial.println(buffer);
    // Serial.print("HX711 reading: ");
    // Serial.print(reading);
    // Serial.print(",");
    // Serial.print(reading2);
    // Serial.print(",");
    // Serial.println(reading3);
  } else {
    Serial.println("HX711 not found.");
  }
  idx++;
  delay(500);
  
}