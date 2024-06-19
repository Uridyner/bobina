constexpr uint8_t sensorPins[5] = { A7, A6, A5, A4, A3 };
constexpr size_t nSensors = sizeof(sensorPins) / sizeof(sensorPins[0]);

constexpr float voltsMaxADC = 5.0;
constexpr uint8_t resADC = 10;
constexpr uint8_t valorMaxADC = pow(2, resADC);

void setup() {
  for (int i = 0; i < nSensors; i++) {
    pinMode(sensorPins[i], INPUT);
  }
  
  Serial.begin(115200);

  analogReadResolution(resADC);
}

void loop() {
  for (int i = 0; i < nSensors; i++) {
    float porcentaje = (float)analogRead(sensorPins[i]) / (float)valorMaxADC;
    Serial.print("Sensor ");
    Serial.print(i);
    Serial.print(":");
    Serial.print(porcentaje * maxVoltajeADC, 3);
    Serial.print('\t');
  }
  Serial.print("Máximo:");
  Serial.print(maxVoltajeADC, 3);
  Serial.print('\t');
  Serial.print("Mínimo:");
  Serial.print(0.0, 3);
  Serial.println();
  delay(1000 / 20);
}
