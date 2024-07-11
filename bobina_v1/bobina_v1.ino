constexpr uint8_t MOT_L_A = 7;
constexpr uint8_t MOT_L_B = 6;
constexpr uint8_t MOT_L_PWM = 11;
constexpr uint8_t MOT_R_A = 9;
constexpr uint8_t MOT_R_B = 12;
constexpr uint8_t MOT_R_PWM = 10;

constexpr uint8_t PINES_SHARPS[3] = { A7, A5, A3 };
constexpr size_t NUM_SHARPS = sizeof(PINES_SHARPS) / sizeof(PINES_SHARPS[0]);

constexpr float VOLTAJE_MAX_ADC = 5.0;
constexpr uint8_t BITS_ADC = 10;
constexpr uint16_t VALOR_MAX_ADC = pow(2, BITS_ADC);

constexpr uint8_t PINES_LEDS[] = { -1, -2 };
constexpr size_t NUM_LEDS = sizeof(PINES_LEDS) / sizeof(PINES_LEDS[0]);

constexpr unsigned int TIEMPO_ESPERA_MS = 5000;

#define DEBUG 1

#if DEBUG
#define debugSetup() Serial.begin(115200)
#define debugPrint(texto) Serial.print(texto)
#define debugPrintln(texto) Serial.println(texto)
#else
#define debugSetup()
#define debugSetup(baudios)
#define debugPrint(texto)
#define debugPrintln(texto)
#endif

void adelante() {
  debugPrintln("Adelante");
  digitalWrite(MOT_L_A, HIGH);
  digitalWrite(MOT_L_B, LOW);
  digitalWrite(MOT_R_A, HIGH);
  digitalWrite(MOT_R_B, LOW);
}

void atras() {
  debugPrintln("Atras");
  digitalWrite(MOT_L_A, LOW);
  digitalWrite(MOT_L_B, HIGH);
  digitalWrite(MOT_R_A, LOW);
  digitalWrite(MOT_R_B, HIGH);
}

void izquierda() {
  debugPrintln("Izquierda");
  digitalWrite(MOT_L_A, HIGH);
  digitalWrite(MOT_L_B, LOW);
  digitalWrite(MOT_R_A, LOW);
  digitalWrite(MOT_R_B, HIGH);
}

void derecha() {
  debugPrintln("Derecha");
  digitalWrite(MOT_L_A, LOW);
  digitalWrite(MOT_L_B, HIGH);
  digitalWrite(MOT_R_A, HIGH);
  digitalWrite(MOT_R_B, LOW);
}

void parada() {
  debugPrintln("Parada");
  digitalWrite(MOT_L_A, LOW);
  digitalWrite(MOT_L_B, LOW);
  digitalWrite(MOT_R_A, LOW);
  digitalWrite(MOT_R_B, LOW);
}

void frenar() {
  debugPrintln("Frenado");
  digitalWrite(MOT_L_A, HIGH);
  digitalWrite(MOT_L_B, HIGH);
  digitalWrite(MOT_R_A, HIGH);
  digitalWrite(MOT_R_B, HIGH);
}

void setupMotores() {
  debugPrintln("Inicializando pines de los motores...");
  debugPrintln("Pin motor izq adelante");
  pinMode(MOT_L_A, OUTPUT);
  debugPrintln("Pin motor izq atras");
  pinMode(MOT_L_B, OUTPUT);
  debugPrintln("Pin motor izq velocidad (PWM)");
  pinMode(MOT_L_PWM, OUTPUT);
  debugPrintln("Pin motor der adelante");
  pinMode(MOT_R_A, OUTPUT);
  debugPrintln("Pin motor der atras");
  pinMode(MOT_R_B, OUTPUT);
  debugPrintln("Pin motor der velocidad (PWM)");
  pinMode(MOT_R_PWM, OUTPUT);
  debugPrintln("Pines de los motores inicializados!");

  debugPrintln("Poniendo dirección inicial en los motores");
  parada();
  debugPrintln("Puesta la dirección inicial en los motores!");

  debugPrintln("Poniendo velocidad en los motores");
  analogWrite(MOT_L_PWM, 255);
  analogWrite(MOT_R_PWM, 255);
  debugPrintln("Velocidad de los motores establecidas!");
}

uint8_t distanciasSharps[NUM_SHARPS];

uint8_t* leerSharps() {
  for (int i = 0; i < NUM_SHARPS; i++) {
    distanciasSharps[i] = (uint8_t)round(17569.7 * pow(analogRead(PINES_SHARPS[i]), -1.2062));
    debugPrint("Sharp ");
    debugPrint(i + 1);
    debugPrint(':');
    debugPrint(distanciasSharps[i]);
    debugPrint('\t');
  }
  debugPrintln();
  return distanciasSharps;
}

void setupSharps() {
  debugPrintln("Inicializando pines de los sharps...");
  for (int i = 0; i < NUM_SHARPS; i++) {
    debugPrint("Sensor distancia sharp ");
    pinMode(PINES_SHARPS[i], INPUT);
    debugPrintln(i + 1);
  }
  debugPrintln("Pines de los sharps inicializados!");
}

void setupLeds() {
  debugPrintln("Inicializando pines de los leds...");
  for (int i = 0; i < NUM_LEDS; i++) {
    debugPrint("Led ");
    pinMode(PINES_LEDS[i], OUTPUT);
    debugPrintln(i + 1);
  }
  debugPrintln("Pines de los leds inicializados!");
}

void cambiarLed(size_t led, bool estado) {
  debugPrintln("Inicializando pines de los leds...");
  for (int i = 0; i < NUM_LEDS; i++) {
    debugPrint("Led ");
    pinMode(PINES_LEDS[i], OUTPUT);
    debugPrintln(i + 1);
  }
  debugPrintln("Pines de los leds inicializados!");
}

void setup() {
  debugSetup();

  cambiarLed(0, false);
  cambiarLed(1, false);
  setupMotores();

  cambiarLed(0, false);
  cambiarLed(1, true);
  setupSharps();

  cambiarLed(0, true);
  cambiarLed(1, false);
  setupLeds();

  cambiarLed(0, true);
  cambiarLed(1, true);

  debugPrint("Comenzando loop en ");
#if DEBUG
  char bufferTextoTiempoEspera[32];
  sprintf(bufferTextoTiempoEspera, "%d.%3d", TIEMPO_ESPERA_MS / 1000, TIEMPO_ESPERA_MS % 1000);
  debugPrint(bufferTextoTiempoEspera);
#endif
  debugPrintln(" segundos...");

  delay(TIEMPO_ESPERA_MS);

  cambiarLed(0, false);
  cambiarLed(1, false);
}

void loop() {
  leerSharps();
  if (distanciasSharps[1] < 60) {
    adelante();
  } else if (distanciasSharps[0] < 60) {
    izquierda();
  } else if (distanciasSharps[2] < 60) {
    derecha();
  } else {
    parada()
  }
  delay(10);
}