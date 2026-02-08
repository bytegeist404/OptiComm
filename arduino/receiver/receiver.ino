const int PHOTO_PIN = A0;

void setup() {
  Serial.begin(115200);
}

void loop() {
  uint16_t v = analogRead(A0);
  uint32_t t = micros();

  Serial.write(0xAA);
  Serial.write((uint8_t*)&t, 4);
  Serial.write((uint8_t*)&v, 2);
}
