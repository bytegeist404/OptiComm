const int PHOTO_PIN = A0;

void setup() {
  Serial.begin(115200);
  // analogReference(INTERNAL);
  pinMode(PHOTO_PIN, INPUT);
}

void loop() {
  uint16_t v = analogRead(A0);  // 0–1023
  Serial.write(0xAA);          // sync byte
  Serial.write((uint8_t*)&v, 2);
  delay(10);
}
