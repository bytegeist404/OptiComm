const int LASER_PIN = 8;


void setup() {
  Serial.begin(115200);
  pinMode(LASER_PIN, OUTPUT);
  digitalWrite(LASER_PIN, LOW);
}

void loop() {
  if (Serial.available()) {
    if (Serial.read() & 1) {
      digitalWrite(LASER_PIN, HIGH);
    } else {  
      digitalWrite(LASER_PIN, LOW);
    }
    
  }
}