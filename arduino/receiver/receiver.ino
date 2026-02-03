const int PHOTO_PIN = A0;

void setup() {
  Serial.begin(115200);
}

void loop() {
  uint16_t v = analogRead(A0);
  uint32_t t = micros();


  uint8_t cksum = 0;
  cksum ^= (uint8_t)(t);
  cksum ^= (uint8_t)(t >> 8);
  cksum ^= (uint8_t)(t >> 16);
  cksum ^= (uint8_t)(t >> 24);
  cksum ^= (uint8_t)(v);
  cksum ^= (uint8_t)(v >> 8);

  Serial.write(0xAA);
  Serial.write((uint8_t*)&t, 4);
  Serial.write((uint8_t*)&v, 2);
  Serial.write(cksum);
}
