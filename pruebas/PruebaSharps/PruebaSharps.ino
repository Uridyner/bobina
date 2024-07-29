constexpr uint8_t PINES_SHARPS[3] = { A7, A5, A3 };
constexpr size_t NUM_SHARPS = sizeof(PINES_SHARPS) / sizeof(PINES_SHARPS[0]);

constexpr float VOLTAJE_MAX_ADC = 5.0;
constexpr uint8_t BITS_ADC = 10;
constexpr uint16_t VALOR_MAX_ADC = pow(2, BITS_ADC);




uint8_t distanciasSharps[NUM_SHARPS];

uint8_t* leerSharps() {
  for (int i = 0; i < NUM_SHARPS; i++) {
    distanciasSharps[i] = (uint8_t)round(17569.7 * pow(analogRead(PINES_SHARPS[i]), -1.2062));
    Serial.print("Sharp ");
    Serial.print(i + 1);
    Serial.print(':');
    Serial.print(distanciasSharps[i]);
    Serial.print('\t');
  }
  Serial.println();
  return distanciasSharps;
}

void setupSharps() {
  Serial.println("Inicializando pines de los sharps...");
  for (int i = 0; i < NUM_SHARPS; i++) {
    Serial.print("Sensor distancia sharp ");
    pinMode(PINES_SHARPS[i], INPUT);
    Serial.println(i + 1);
  }
  Serial.println("Pines de los sharps inicializados!");
}

void setup() {
  Serial.begin(115200); // start the serial port
    setupSharps();
}

void loop() {
  leerSharps();
}