const int LASER_PIN = 13;

const int BIT_DURATION_MS = 40;

void sendBit(bool bit) {
  digitalWrite(LASER_PIN, bit ? HIGH : LOW);
  delay(BIT_DURATION_MS);
}

void sendByte(uint8_t b) {
  sendBit(1);  // start bit

  for (int i = 0; i < 8; i++) {
    sendBit((b >> i) & 1);
  }


  sendBit(0);  // stop bit

  delay(BIT_DURATION_MS * 2);
}

void setup() {
  Serial.begin(115200);
  pinMode(LASER_PIN, OUTPUT);
  digitalWrite(LASER_PIN, LOW);

  Serial.println("Laser Sender Ready");
}

void loop() {
  if (Serial.available()) {
    uint8_t b = Serial.read();
    Serial.print("Sending byte: ");
    Serial.print(b);
    Serial.print(" (");
    Serial.print((char)b);
    Serial.println(")");
    sendByte(b);
  }
}
