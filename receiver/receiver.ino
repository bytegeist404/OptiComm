const int PHOTO_PIN = A0;

const int BIT_DURATION_MS = 40;
const float SIGNAL_THRESHOLD = 12;

float background = 0;
float photo_val = 0;

float readSignal() {
  photo_val = analogRead(PHOTO_PIN);
  background = background * 0.98 + photo_val * 0.02;
  return photo_val - background;
}

bool readBit() {
  unsigned long start = millis();
  while (millis() - start < BIT_DURATION_MS) continue;
  if (readSignal() > SIGNAL_THRESHOLD) return true;
  return false;
}

uint8_t receiveByte() {
  readBit();  // start bit
  uint8_t b = 0;

  for (int i = 0; i < 8; i++) {
    if (readBit()) b |= (1 << i);
  }

  readBit();  // stop bit
  return b;
}

void setup() {
  Serial.begin(115200);
  pinMode(PHOTO_PIN, INPUT);

  Serial.println("Laser Receiver Ready");
}

void loop() {
  float sig = readSignal();
  if (sig > SIGNAL_THRESHOLD) {
    uint8_t b = receiveByte();

    Serial.print("Received: ");
    Serial.print(b);
    Serial.print(" (");
    Serial.print((char)b);
    Serial.println(")");
  }
}
