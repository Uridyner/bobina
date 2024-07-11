constexpr uint8_t MOT_L_A = 7;
constexpr uint8_t MOT_L_B = 6;
constexpr uint8_t MOT_L_PWM = 11;
constexpr uint8_t MOT_R_A = 9;
constexpr uint8_t MOT_R_B = 12;
constexpr uint8_t MOT_R_PWM = 10;

constexpr uint8_t PINES_SHARPS[3] = { A7, A6, A5 };
constexpr size_t NUM_SHARPS = sizeof(PINES_SHARPS) / sizeof(PINES_SHARPS[0]);

constexpr float VOLTAJE_MAX_ADC = 5.0;
constexpr uint8_t BITS_ADC = 10;
constexpr uint16_t VALOR_MAX_ADC = pow(2, BITS_ADC);

void adelante() {
  Serial.println("Adelante");
  digitalWrite(MOT_L_A, HIGH);
  digitalWrite(MOT_L_B, LOW);
  digitalWrite(MOT_R_A, HIGH);
  digitalWrite(MOT_R_B, LOW);
}

void atras() {
  Serial.println("Atras");
  digitalWrite(MOT_L_A, LOW);
  digitalWrite(MOT_L_B, HIGH);
  digitalWrite(MOT_R_A, LOW);
  digitalWrite(MOT_R_B, HIGH);
}

void izquierda() {
  Serial.println("Izquierda");
  digitalWrite(MOT_L_A, HIGH);
  digitalWrite(MOT_L_B, LOW);
  digitalWrite(MOT_R_A, LOW);
  digitalWrite(MOT_R_B, HIGH);
}

void derecha() {
  Serial.println("Derecha");
  digitalWrite(MOT_L_A, LOW);
  digitalWrite(MOT_L_B, HIGH);
  digitalWrite(MOT_R_A, HIGH);
  digitalWrite(MOT_R_B, LOW);
}

void parada() {
  Serial.println("Parada");
  digitalWrite(MOT_L_A, LOW);
  digitalWrite(MOT_L_B, LOW);
  digitalWrite(MOT_R_A, LOW);
  digitalWrite(MOT_R_B, LOW);
}

void frenar() {
  Serial.println("Frenado");
  digitalWrite(MOT_L_A, HIGH);
  digitalWrite(MOT_L_B, HIGH);
  digitalWrite(MOT_R_A, HIGH);
  digitalWrite(MOT_R_B, HIGH);
}

void setupMotores() {
  Serial.println("Inicializando pines de los motores...");
  Serial.println("Pin motor izq adelante");
  pinMode(MOT_L_A, OUTPUT);
  Serial.println("Pin motor izq atras");
  pinMode(MOT_L_B, OUTPUT);
  Serial.println("Pin motor izq velocidad (PWM)");
  pinMode(MOT_L_PWM, OUTPUT);
  Serial.println("Pin motor der adelante");
  pinMode(MOT_R_A, OUTPUT);
  Serial.println("Pin motor der atras");
  pinMode(MOT_R_B, OUTPUT);
  Serial.println("Pin motor der velocidad (PWM)");
  pinMode(MOT_R_PWM, OUTPUT);
  Serial.println("Pines de los motores inicializados!");

  Serial.println("Poniendo dirección inicial en los motores");
  parada();
  Serial.println("Puesta la dirección inicial en los motores!");

  Serial.println("Poniendo velocidad en los motores");
  analogWrite(MOT_L_PWM, 255);
  analogWrite(MOT_R_PWM, 255);
  Serial.println("Velocidad de los motores establecidas!");
}

uint8_t distanciasSharps[NUM_SHARPS];

uint8_t* leerSharps() {
  for (int i = 0; i < NUM_SHARPS; i++) {
    distanciasSharps[i] = (uint8_t)round(17569.7 * pow(analogRead(PINES_SHARPS[i]), -1.2062));
    Serial.print("Sharp ");
    Serial.print(i+1);
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
    Serial.println(i+1);
  }
  Serial.println("Pines de los sharps inicializados!");
}

void setup() {
  Serial.begin(115200);

  setupMotores();

  setupSharps();

  Serial.println("Comenzando loop en 2 segundos...");
  delay(2000);
}

void loop() {
  leerSharps();
}