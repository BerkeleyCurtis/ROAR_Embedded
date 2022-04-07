#define IR 25

int reading;

void setup() {
  Serial.begin(115200);
}

void loop() {
  reading = analogRead(IR);

  Serial.println(reading);

  delay(500);
}