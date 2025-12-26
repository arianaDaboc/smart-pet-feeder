//cod calculare factor calibrare;;;; factor = (RAW_cu_greutate − RAW_fara_greutate) / greutate_cunoscuta
#include "HX711.h"

#define DT 5
#define SCK 4

HX711 scale;

void setup() {
  Serial.begin(57600);
  scale.begin(DT, SCK);

  Serial.println("=== CALIBRATION MODE ===");
  Serial.println("1. Leave the scale EMPTY");
  delay(5000);

  scale.tare();
  long zero = scale.read();
  Serial.print("RAW_ZERO = ");
  Serial.println(zero);

  Serial.println("2. Put EXACT 193g on the scale");
  delay(8000);

  long raw = scale.read();
  Serial.print("RAW_193g = ");
  Serial.println(raw);

  Serial.println("=== END ===");
}

void loop() {}