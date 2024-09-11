#define LED_AM 4
#define LED_A1 5
#define LED_A2 8

void setup() {
  pinMode(LED_AM, OUTPUT);
  pinMode(LED_A1, OUTPUT);
  pinMode(LED_A2, OUTPUT);
}

void loop() {
  digitalWrite(LED_AM, HIGH);
  digitalWrite(LED_A1, LOW);
  digitalWrite(LED_A2, HIGH);
}
