#include <Smoothed.h>

/// Motor izquierdo, adelante
constexpr uint8_t MOT_L_A = 7;
/// Motor izquierdo, atrás
constexpr uint8_t MOT_L_B = 6;
/// Motor izquierdo, velocidad/PWM
constexpr uint8_t MOT_L_PWM = 11;
/// Motor izquierdo, velocidad/PWM máximo
constexpr uint8_t MOT_L_PWM_MAX = 255;
/// Motor derecho, adelante
constexpr uint8_t MOT_R_A = 9;
/// Motor derecho, atrás
constexpr uint8_t MOT_R_B = 12;
/// Motor derecho, velocidad/PWM
constexpr uint8_t MOT_R_PWM = 10;
/// Motor derecho, velocidad/PWM máximo
constexpr uint8_t MOT_R_PWM_MAX = 255;

enum LadoSharp {
  SHARP_IZQ = 0,
  SHARP_CEN = 1,
  SHARP_DER = 2,
};
/// Pines de los sharps
constexpr uint8_t PINES_SHARPS[] = { A7, A5, A3 };
/// Numero de sharps en la placa
constexpr size_t NUM_SHARPS = sizeof(PINES_SHARPS) / sizeof(PINES_SHARPS[0]);
/// Distancia que usan los sharps para saber que detectan algo
constexpr unsigned int DISTANCIA_ACTIVACION_SHARPS = 35;
/// Cantidad de veces que se leen los sharps.
///
/// Se usa para tomar un promedio de las lecturas y limpiar los valores.
constexpr size_t LECTURAS_SHARP = 1;
/// Lecturas por segundos de los sharp
constexpr unsigned long FRECUENCIA_LECTURA_SHARP = 40;

enum LadoCNY {
  CNY_IZQ = 0,
  CNY_DER = 1,
};
/// Pines de los CNYs
constexpr uint8_t PINES_CNY[] = { A6, A4 };
/// Numero de CNYs en la placa
constexpr size_t NUM_CNY = sizeof(PINES_CNY) / sizeof(PINES_SHARPS[0]);
constexpr float ACTIVACIONES_CNY[NUM_CNY] = { 0.4, 0.4 };
/// Cantidad de veces que se leen los CNYs.
///
/// Se usa para tomar un promedio de las lecturas y limpiar los valores.
constexpr size_t LECTURAS_CNY = 3;

/// Voltaje máximo del ADC
constexpr float VOLTAJE_MAX_ADC = 5.0;
/// Bits que tiene el ADC
constexpr uint8_t BITS_ADC = 10;
/// Valor máximo del ADC
constexpr uint16_t VALOR_MAX_ADC = 1024;
/// Convertir lectura a voltaje
constexpr float lecturaAVoltaje(uint16_t lectura) {
  return ((float)lectura * VOLTAJE_MAX_ADC) / (float)VALOR_MAX_ADC;
}
/// Convertir voltaje a lectura
constexpr uint16_t voltajeALectura(float voltaje) {
  return (uint16_t)((voltaje * (float)VALOR_MAX_ADC) / VOLTAJE_MAX_ADC);
}

/// Pines de los LEDs
constexpr uint8_t PINES_LEDS[] = { 5, 8, 4 };
/// Numero de LEDs en la placa
constexpr size_t NUM_LEDS = sizeof(PINES_LEDS) / sizeof(PINES_LEDS[0]);

/// Pines de los botones
constexpr uint8_t PINES_BOTONES[] = { 2, 3 };
/// Numero de botones en la placa
constexpr size_t NUM_BOTONES = sizeof(PINES_BOTONES) / sizeof(PINES_BOTONES[0]);

/// Tiempo que se espera antes de empezar a correr
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
  frenar();
  debugPrintln("Puesta la dirección inicial en los motores!");

  debugPrintln("Poniendo velocidad en los motores");
  analogWrite(MOT_L_PWM, MOT_L_PWM_MAX);
  analogWrite(MOT_R_PWM, MOT_R_PWM_MAX);
  debugPrintln("Velocidad de los motores establecidas!");
}

Smoothed<unsigned int> sharps[NUM_SHARPS];
unsigned long ultimaLecturaSharps = -1;

Smoothed<unsigned int>* leerSharps() {
  if (millis() - ultimaLecturaSharps < 1000 / FRECUENCIA_LECTURA_SHARP) {
    return sharps;
  }
  ultimaLecturaSharps = millis();
  for (size_t i = 0; i < NUM_SHARPS; i++) {
    const unsigned int distancia = (unsigned int)round(17569.7 * pow(analogRead(PINES_SHARPS[i]), -1.2062));
    sharps[i].add(distancia);
    debugPrint("Sharp");
    debugPrint(i + 1);
    debugPrint(':');
    debugPrint(sharps[i].get());
    debugPrint('\t');
  }
  debugPrintln();
  ultimaLecturaSharps = millis();
  return sharps;
}

void setupSharps() {
  debugPrintln("Inicializando pines de los sharps...");
  for (int i = 0; i < NUM_SHARPS; i++) {
    debugPrint("Sensor distancia sharp ");
    pinMode(PINES_SHARPS[i], INPUT);
	  sharps[i].begin(SMOOTHED_AVERAGE, LECTURAS_SHARP);	
    debugPrintln(i + 1);
  }
  debugPrintln("Pines de los sharps inicializados!");
}

float lecturasCNY[NUM_CNY];

float* leerCNY() {
  for (size_t i = 0; i < NUM_CNY; i++) {
    lecturasCNY[i] = 0;
    for (size_t j = 0; j < LECTURAS_CNY; j++) {
      lecturasCNY[i] += lecturaAVoltaje(analogRead(PINES_CNY[i])) / LECTURAS_CNY;
    }
    debugPrint("CNY");
    debugPrint(i + 1);
    debugPrint(':');
    debugPrint(lecturasCNY[i]);
    debugPrint('\t');
  }
  debugPrintln();
  return lecturasCNY;
}

void setupCNY() {
  debugPrintln("Inicializando pines de los CNYs...");
  for (int i = 0; i < NUM_CNY; i++) {
    debugPrint("Sensor piso CNY ");
    pinMode(PINES_CNY[i], INPUT);
    debugPrintln(i + 1);
  }
  debugPrintln("Pines de los CNYs inicializados!");
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

inline void cambiarLed(size_t led, bool estado) {
  digitalWrite(PINES_LEDS[led], estado);
}

void setupBotones() {
  debugPrintln("Inicializando pines de los botones...");
  for (int i = 0; i < NUM_BOTONES; i++) {
    debugPrint("Boton ");
    pinMode(PINES_BOTONES[i], INPUT_PULLUP);
    debugPrintln(i + 1);
  }
  debugPrintln("Pines de los botones inicializados!");
}

inline bool estaPresionado(size_t boton) {
  return digitalRead(PINES_BOTONES[boton]) == LOW;
}

enum {
  GIRO_DER,
  GIRO_IZQ,
} giroPreferido;

void setup() {
  debugSetup();

  for (size_t i = 0; i < NUM_LEDS - 1; i++) {
    cambiarLed(i, bitRead(0, i));
  }
  setupMotores();

  for (size_t i = 0; i < NUM_LEDS - 1; i++) {
    cambiarLed(i, bitRead(1, i));
  }
  setupSharps();

  for (size_t i = 0; i < NUM_LEDS - 1; i++) {
    cambiarLed(i, bitRead(2, i));
  }
  setupLeds();

  for (size_t i = 0; i < NUM_LEDS - 1; i++) {
    cambiarLed(i, bitRead(3, i));
  }
  setupCNY();

  for (size_t i = 0; i < NUM_LEDS - 1; i++) {
    cambiarLed(i, bitRead(4, i));
  }
  setupBotones();

  unsigned long ultimoCambioLeds = millis();

  while (estaPresionado(0) == false && estaPresionado(1) == false) {
    if (millis() - ultimoCambioLeds > 125) {
      for (size_t i = 0; i < NUM_LEDS - 1; i++) {
        cambiarLed(i, rand() % 2);
      }
      ultimoCambioLeds = millis();
    }
  }

  if (estaPresionado(0)) {
    giroPreferido = GIRO_IZQ;
  } else if (estaPresionado(1)) {
    giroPreferido = GIRO_DER;
  }

  unsigned long tiempoComienzo = millis();

#if DEBUG
  unsigned char ultimoDigitoTiempoAnterior = -1;
#endif

  while (millis() - tiempoComienzo < TIEMPO_ESPERA_MS) {
    unsigned long segundosRestantes = (TIEMPO_ESPERA_MS - (millis() - tiempoComienzo)) / 1000;

    for (size_t i = 0; i < NUM_LEDS - 1; i++) {
      cambiarLed(i, bitRead(segundosRestantes, i) == true);
    }
    cambiarLed(NUM_LEDS - 1, giroPreferido == GIRO_IZQ);

#if DEBUG
    const unsigned char ultimoDigitoTiempo = ((TIEMPO_ESPERA_MS - (millis() - tiempoComienzo)) / 100);
    char pluralSegundos = "s";
    if (segundosRestantes == 1 && ultimoDigitoTiempo == 0) {
      pluralSegundos = 0;
    }
    char bufferTextoTiempoEspera[128];
    sprintf(bufferTextoTiempoEspera, "Comenzando en %d.%1d segundo%c", segundosRestantes, ultimoDigitoTiempo);
    debugPrintln(bufferTextoTiempoEspera);
    ultimoDigitoTiempoAnterior = ultimoDigitoTiempo;
#endif
  }

  cambiarLed(0, giroPreferido == GIRO_IZQ);
  cambiarLed(1, giroPreferido == GIRO_DER);
}

enum {
  ATRAS_NADA,
  ATRAS_RECTO,
  ATRAS_IZQ,
  ATRAS_DER,
} retrocediendo;
unsigned long ultimoCambioRetrocediendo = -1;

void loop() {
  leerCNY();
  if (millis() - ultimoCambioRetrocediendo > 700) {
    retrocediendo = ATRAS_NADA;
  }
  if (retrocediendo == ATRAS_NADA) {
    if (lecturasCNY[CNY_IZQ] <= ACTIVACIONES_CNY[CNY_IZQ] && lecturasCNY[CNY_DER] <= ACTIVACIONES_CNY[CNY_DER]) {
      retrocediendo = ATRAS_RECTO;
      ultimoCambioRetrocediendo = millis();
    } else if (lecturasCNY[CNY_IZQ] <= ACTIVACIONES_CNY[CNY_IZQ] && lecturasCNY[CNY_DER] > ACTIVACIONES_CNY[CNY_DER]) {
      retrocediendo = ATRAS_DER;
      ultimoCambioRetrocediendo = millis();
    } else if (lecturasCNY[CNY_IZQ] > ACTIVACIONES_CNY[CNY_IZQ] && lecturasCNY[CNY_DER] <= ACTIVACIONES_CNY[CNY_DER]) {
      retrocediendo = ATRAS_IZQ;
      ultimoCambioRetrocediendo = millis();
    }
  }
  leerSharps();
  if (sharps[SHARP_CEN].get() < DISTANCIA_ACTIVACION_SHARPS && retrocediendo == ATRAS_NADA) {
    analogWrite(MOT_L_PWM, MOT_L_PWM_MAX);
    analogWrite(MOT_R_PWM, MOT_R_PWM_MAX);
    adelante();
  } else if (sharps[SHARP_IZQ].get() < DISTANCIA_ACTIVACION_SHARPS) {
    analogWrite(MOT_L_PWM, 3 * (MOT_L_PWM_MAX / 4));
    analogWrite(MOT_R_PWM, 3 * (MOT_R_PWM_MAX / 4));
    izquierda();
    retrocediendo = ATRAS_NADA;
  } else if (sharps[SHARP_DER].get() < DISTANCIA_ACTIVACION_SHARPS) {
    analogWrite(MOT_L_PWM, 3 * (MOT_L_PWM_MAX / 4));
    analogWrite(MOT_R_PWM, 3 * (MOT_R_PWM_MAX / 4));
    derecha();
    retrocediendo = ATRAS_NADA;
  } else {
    switch (retrocediendo) {
      case ATRAS_RECTO:
        analogWrite(MOT_L_PWM, MOT_L_PWM_MAX);
        analogWrite(MOT_R_PWM, MOT_R_PWM_MAX);
        atras();
        break;
      case ATRAS_DER:
        analogWrite(MOT_L_PWM, 3 * (MOT_L_PWM_MAX / 4));
        analogWrite(MOT_R_PWM, MOT_R_PWM_MAX);
        atras();
        break;
      case ATRAS_IZQ:
        analogWrite(MOT_L_PWM, MOT_L_PWM_MAX);
        analogWrite(MOT_R_PWM, 3 * (MOT_R_PWM_MAX / 4));
        atras();
        break;
      default:
        analogWrite(MOT_L_PWM, MOT_L_PWM_MAX);
        analogWrite(MOT_R_PWM, MOT_R_PWM_MAX);
        if (giroPreferido == GIRO_DER) {
          derecha();
        } else if (giroPreferido == GIRO_IZQ) {
          izquierda();
        }
    }
  }
  for (size_t i = 0; i < NUM_LEDS; i++) {
    cambiarLed(i, bitRead(sharps[SHARP_CEN].get() >> 2, i) == true);
  }
#ifdef DEBUG
  delay(100);
#endif
}