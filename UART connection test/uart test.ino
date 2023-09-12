void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial1.begin(9600, SERIAL_8N1, 18, 17);
}

void loop() {
  // put your main code here, to run repeatedly:
  while (Serial1.available()) {
    Serial.print(char(Serial1.read()));
  }
}
