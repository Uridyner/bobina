#include <Smoothed.h>

/// Motor izquierdo, adelante
constexpr uint8_t MOT_L_A = 6;
/// Motor izquierdo, atrás
constexpr uint8_t MOT_L_B = 7;
/// Motor izquierdo, velocidad/PWM
constexpr uint8_t MOT_L_PWM = 11;
/// Motor izquierdo, velocidad/PWM máximo
constexpr uint8_t MOT_L_PWM_MAX = 150;
/// Motor derecho, adelante
constexpr uint8_t MOT_R_A = 12;
/// Motor derecho, atrás
constexpr uint8_t MOT_R_B = 9;
/// Motor derecho, velocidad/PWM
constexpr uint8_t MOT_R_PWM = 10;
/// Motor derecho, velocidad/PWM máximo
constexpr uint8_t MOT_R_PWM_MAX = 150;

enum LadoSharp {
  SHARP_IZQ = 0,
  SHARP_CEN = 1,
  SHARP_DER = 2,
};
/// Pines de los sharps
constexpr uint8_t PINES_SHARPS[] = { A7, A6, A5 };
/// Numero de sharps en la placa
constexpr size_t NUM_SHARPS = sizeof(PINES_SHARPS) / sizeof(PINES_SHARPS[0]);
/// Cantidad de veces que se leen los sharps.
///
/// Se usa para tomar un promedio de las lecturas y limpiar los valores.
constexpr size_t LECTURAS_SHARP = 5;
/// Lecturas por segundos de los sharp
///
/// ! Desde que empezamos a usar la librería SharpIR esto se volvió inutil.S
constexpr unsigned long FRECUENCIA_LECTURA_SHARP = 30;

enum LadoCNY {
  CNY_IZQ = 0,
  CNY_DER = 1,
};
/// Pines de los CNYs
constexpr uint8_t PINES_CNY[] = { A3, A4 };
/// Numero de CNYs en la placa
constexpr size_t NUM_CNY = sizeof(PINES_CNY) / sizeof(PINES_SHARPS[0]);
/// Cantidad de veces que se leen los CNYs.
///
/// Se usa para tomar un promedio de las lecturas y limpiar los valores.
constexpr size_t LECTURAS_CNY = 1;

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
constexpr uint8_t PINES_LEDS[] = { 8, 5, 4 };
/// Numero de LEDs en la placa
constexpr size_t NUM_LEDS = sizeof(PINES_LEDS) / sizeof(PINES_LEDS[0]);

/// Pines de los botones
constexpr uint8_t PINES_BOTONES[] = { 2, 3 };
/// Numero de botones en la placa
constexpr size_t NUM_BOTONES = sizeof(PINES_BOTONES) / sizeof(PINES_BOTONES[0]);

/// Tiempo que se espera antes de empezar a correr
constexpr unsigned int TIEMPO_ESPERA_MS = 5000;

/// Tiempo que retrocede cuando detecta que está sobre el borde
constexpr unsigned int TIEMPO_RETROCEDER_MS = 200;

/// Tiempo que se espera antes de avanzar forzadamente cuando está girando
/// para evitar que se pare la pelea
constexpr unsigned int TIEMPO_ESPERA_AVANCE_FORZADO_MS = 500;

/// Tiempo que avanza forzadamente para evitar que se pare la pelea
constexpr unsigned int TIEMPO_AVANCE_FORZADO_MS = 350;

// #define DEBUG 1

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

unsigned long activacionesSharp[NUM_SHARPS] = { 55, 55, 55 };
Smoothed<uint16_t> smoothedSharps[NUM_SHARPS];
unsigned long ultimaLecturaSharps = -1;

Smoothed<unsigned int>* leerSharps() {
  if (millis() - ultimaLecturaSharps >= 1000 / FRECUENCIA_LECTURA_SHARP) {
    ultimaLecturaSharps = millis();
    for (size_t i = 0; i < NUM_SHARPS; i++) {
      uint16_t lectura = analogRead(PINES_SHARPS[i]);
      smoothedSharps[i].add(lectura);
      debugPrint("Sharp");
      debugPrint(i + 1);
      debugPrint(':');
      debugPrint(lectura);
      debugPrint('\t');
    }
    debugPrintln();
    ultimaLecturaSharps = millis();
  }
  return smoothedSharps;
}

void setupSharps() {
  debugPrintln("Inicializando pines de los sharps...");
  for (int i = 0; i < NUM_SHARPS; i++) {
    debugPrint("Sensor distancia sharp ");
    pinMode(PINES_SHARPS[i], INPUT);
    smoothedSharps[i].begin(SMOOTHED_AVERAGE, LECTURAS_SHARP);
    debugPrintln(i + 1);
  }
  debugPrintln("Pines de los sharps inicializados!");
}

float activacionesCNY[NUM_CNY] = { 0, 0 };
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
  bool valor = digitalRead(PINES_BOTONES[boton]);
  debugPrint("Boton ");
  debugPrint(boton);
  debugPrint(": ");
  debugPrintln(valor);
  if (boton == 0) {
    return valor == HIGH;
  } else if (boton == 1) {
    return valor == LOW;
  }
}

enum {
  BASICA_DER,
  BASICA_IZQ,
  PASITOS,
  NUM_ESTRATEGIAS,
} estrategia;

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
  unsigned long ultimaPresionBoton = millis();
  bool ultimoEstadoBoton = estaPresionado(1);

  // Cambiar estado o salir si se mantuvo por 1000 ms
  while (estaPresionado(1) == false && ultimoEstadoBoton == true && millis() - ultimaPresionBoton > 1000) {
    for (size_t i = 0; i < NUM_LEDS - 1; i++) {
      unsigned char valor = (estrategia / (int)pow(3, i)) % 3;
      if (valor == 0) {
        cambiarLed(i, false);
      } else if (valor == 1) {
        cambiarLed(i, (millis() / 64) < 32);
      } else if (valor == 2) {
        cambiarLed(i, true);
      }
    }
    bool estadoBoton = estaPresionado(1);
    if (estadoBoton == true && ultimoEstadoBoton == false) {
      estrategia++;
      estrategia %= NUM_ESTRATEGIAS;
    }
    ultimoEstadoBoton = estadoBoton;
  }

  for (size_t i = 0; i < NUM_LEDS - 1; i++) {
    cambiarLed(i, true);
  }

  // Esperar a que se suelte
  while (estaPresionado(1) == true) {}

  // Esperar a que se aprete
  while (estaPresionado(1) == false) {}

  unsigned long tiempoComienzo = millis();

#if DEBUG
  unsigned char ultimoDigitoTiempoAnterior = -1;
#endif

  unsigned long ultimaLecturaCNYs = millis();
  unsigned int numeroLecturasCNYs = 1;
  unsigned long ultimaLecturaSharps = millis();
  unsigned int numeroLecturasSharps = 1;

  while (millis() - tiempoComienzo < TIEMPO_ESPERA_MS) {
    unsigned long segundosRestantes = (TIEMPO_ESPERA_MS - (millis() - tiempoComienzo)) / 1000;

    for (size_t i = 0; i < NUM_LEDS - 1; i++) {
      cambiarLed(i, bitRead(segundosRestantes, i) == true);
    }
    cambiarLed(NUM_LEDS - 1, giroPreferido == GIRO_IZQ);

    if (millis() - ultimaLecturaCNYs > 20) {
      leerCNY();
      for (size_t i = 0; i < NUM_CNY; i++) {
        activacionesCNY[i] += lecturasCNY[i];
      }
      ultimaLecturaCNYs = millis();
      numeroLecturasCNYs++;
    }

    if (millis() - ultimaLecturaSharps > 40 && segundosRestantes <= 2) {
      for (size_t i = 0; i < NUM_SHARPS; i++) {
        unsigned long lectura = analogRead(PINES_SHARPS[i]);
        activacionesSharp[i] += lectura;
        debugPrint("Sharp");
        debugPrint(i + 1);
        debugPrint(':');
        debugPrint(lectura);
        debugPrint('\t');
      }
      debugPrintln();
      ultimaLecturaSharps = millis();
      numeroLecturasSharps++;
    }
  }

  debugPrintln("Activaciones CNYs");
  for (size_t i = 0; i < NUM_CNY; i++) {
    activacionesCNY[i] /= numeroLecturasCNYs;
    activacionesCNY[i] /= 2;
    debugPrint(activacionesCNY[i]);
    debugPrint('\t');
  }
  debugPrintln();

  debugPrintln("Activaciones Sharps");
  for (size_t i = 0; i < NUM_SHARPS; i++) {
    activacionesSharp[i] /= numeroLecturasSharps;
    activacionesSharp[i] *= 2.3;
    if (i == SHARP_IZQ) {
      activacionesSharp[i] = min(activacionesSharp[i], 180);
    } else {
      activacionesSharp[i] = min(activacionesSharp[i], 80);
    }
    debugPrint(activacionesSharp[i]);
    debugPrint('\t');
  }
  debugPrintln();

#if DEBUG
  delay(1000);
#endif
}

unsigned long ultimoRetroceso = -1;

unsigned long siguienteTiempoAvance = -1;

/// Literalmente solo buscar y matar
void estrategiaBasica(bool girarDerechaPorDefecto) {
  bool sharpIzq = smoothedSharps[SHARP_IZQ].get() > activacionesSharp[SHARP_IZQ];
  bool sharpCen = smoothedSharps[SHARP_CEN].get() > activacionesSharp[SHARP_CEN];
  bool sharpDer = smoothedSharps[SHARP_DER].get() > activacionesSharp[SHARP_DER];

  /// Último tiempo en milisegundos que empece a detectar algo en el sharp del centro
  static unsigned long ultimoTiempoDeteccionCentro = millis();
  /// Si la última vez que corrí detecté algo en el sharp del centro
  static bool ultimoDetectandoCentro = false;

  if (sharpCen) {
    // Si acabo de detectar algo en el sharp del centro
    if (ultimoDetectandoCentro == false) {
      ultimoTiempoDeteccionCentro = millis();
    }
    if (millis() - ultimoTiempoDeteccionCentro > 1000) {
      analogWrite(MOT_L_PWM, 255);
      analogWrite(MOT_R_PWM, 255);
    } else {
      analogWrite(MOT_L_PWM, MOT_L_PWM_MAX);
      analogWrite(MOT_R_PWM, MOT_L_PWM_MAX);
    }
    adelante();
  } else {
    analogWrite(MOT_L_PWM, MOT_L_PWM_MAX);
    analogWrite(MOT_R_PWM, MOT_L_PWM_MAX);
    if (girarDerechaPorDefecto) {
      if (sharpIzq) {
        izquierda();
      } else {
        derecha();
      }
    } else {
      if (sharpDer) {
        derecha();
      } else {
        izquierda();
      }
    }
  }

  ultimoDetectandoAdelante = sharpCen;
}

void estrategiaPasitos() {
  bool sharpIzq = smoothedSharps[SHARP_IZQ].get() > activacionesSharp[SHARP_IZQ];
  bool sharpCen = smoothedSharps[SHARP_CEN].get() > activacionesSharp[SHARP_CEN];
  bool sharpDer = smoothedSharps[SHARP_DER].get() > activacionesSharp[SHARP_DER];
}

bool retrocediendo = false;

void loop() {
  leerSharps();
  debugPrint(activacionesSharp[SHARP_IZQ]);
  debugPrint('\t');
  debugPrint(activacionesSharp[SHARP_CEN]);
  debugPrint('\t');
  debugPrintln(activacionesSharp[SHARP_DER]);

  leerCNY();
  debugPrint(lecturasCNY[CNY_IZQ]);
  debugPrint('\t');
  debugPrintln(lecturasCNY[CNY_DER]);

  bool cnyIzq = lecturasCNY[CNY_IZQ] <= activacionesCNY[CNY_IZQ];
  bool cnyDer = lecturasCNY[CNY_DER] <= activacionesCNY[CNY_DER];
  if (cnyIzq || cnyDer) {
    ultimoCambioRetrocediendo = millis();
    retrocediendo = true;
  }

  if (retrocediendo) {
    if (millis() - ultimoCambioRetrocediendo > TIEMPO_RETROCEDER_MS) {
      retrocediendo = false;
    }
  } else {
    switch (estrategia) {
      case BASICA_DER:
        estrategiaBasica(false);
        break;
      case BASICA_IZQ:
        estrategiaBasica(true);
        break;
      case PASITOS:
        estrategiaBasica(true);
        break;
    }
  }

  cambiarLed(0, sharpIzq);
  cambiarLed(1, sharpCen);
  cambiarLed(2, sharpDer);
  // for (size_t i = 0; i < NUM_LEDS; i++) {
  //   cambiarLed(i, bitRead(smoothedSharps[SHARP_CEN].get() >> 1, i) == true);
  // }
#if DEBUG
  delay(100);
#endif
}
