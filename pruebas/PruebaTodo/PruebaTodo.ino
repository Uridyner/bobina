#include "TimerOne.h"


#define BOTON_1 2
#define BOTON_2 3

#define LED_V 4
#define LED_R1 5
#define LED_R2 8


#define INIT 0
#define E1 1
#define E2 2
#define START 3
int estado = 0;
int mseg = 0;
int estadoLogica = 0;

#define ESPERA 0
#define CONFIRMACION 1
#define LIBERACION 2
#define TIEMPO_antiReboteBoton1 15
int estadoantiReboteBoton1 = 0;
int mSegantiReboteBoton1 = 0;
int flagBoton1 = 0;

#define CNY1 A6
#define CNY2 A4
int valorCNY1 = 0;
int valorCNY2 = 0;

//MOTORES y SHARPS
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

void izquierda2() {
  Serial.println("Izquierda2");
  digitalWrite(MOT_L_A, LOW);
  digitalWrite(MOT_L_B, HIGH);
  digitalWrite(MOT_R_A, LOW);
  digitalWrite(MOT_R_B, LOW);
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
  pinMode(BOTON_1, INPUT_PULLUP);
  pinMode(BOTON_2, INPUT);

  pinMode(LED_V, OUTPUT);
  pinMode(LED_R1, OUTPUT);
  pinMode(LED_R2, OUTPUT);

  Timer1.initialize(1000);
  Timer1.attachInterrupt(callBack);

  Serial.begin(115200);

  setupMotores();

  setupSharps();

  Serial.println("Comenzando loop en 2 segundos...");
  delay(2000);
}

void loop() {
  antiReboteBoton1();
  maquinaBot1();
}



void callBack() {
  mseg = mseg + 1;
  mSegantiReboteBoton1 += 1;
}

void maquinaBot1() {
  switch (estado) {
    case INIT:
      digitalWrite(LED_V, LOW);
      digitalWrite(LED_R1, HIGH);
      digitalWrite(LED_R2, LOW);
      estado = E1;
      break;

    case E1:
      if (flagBoton1 == 1) {
        mseg = 0;
        estado = E2;
        flagBoton1 = 0;
        digitalWrite(LED_R2, HIGH);
      }
      break;

    case E2:

      if (mseg >= 3000) {
        digitalWrite(LED_R1, LOW);
        digitalWrite(LED_R2, LOW);
        mseg = 0;
        estado = START;
      }
      break;
    case START:
      digitalWrite(LED_V, HIGH);
      leerSharps();
      valorCNY1 = analogRead(CNY1);
      valorCNY2 = analogRead(CNY2);
      Serial.println(valorCNY1);
      Serial.println(valorCNY2);

      maquinaLogica();


      break;
  }
}

void maquinaLogica() {
  switch (estadoLogica) {
    case INIT:
      estadoLogica = E1;
      break;

    case E1:

      if (distanciasSharps[0] > 20 || distanciasSharps[1] > 20 || distanciasSharps[2] > 20) {
        parada();
        if (valorCNY1 < 35 || valorCNY2 < 35) {
          estadoLogica = E2;
        }
      }


      if (distanciasSharps[0] < 20 || distanciasSharps[1] < 20 || distanciasSharps[2] < 20) {
        adelante();
        if (valorCNY1 < 35 || valorCNY2 < 35) {
          estadoLogica = E2;
        }
      }

      if (valorCNY1 < 35 || valorCNY2 < 35) {
        estadoLogica = E2;
      }
      break;

    case E2:

      izquierda2();

      if (valorCNY1 > 35 && valorCNY2 > 35) {
        estadoLogica = E1;
      }

      break;
  }
}

void antiReboteBoton1() {

  switch (estadoantiReboteBoton1) {
    case ESPERA:


      if (digitalRead(BOTON_1) == LOW) {
        Serial.println("Apretado");
        mSegantiReboteBoton1 = 0;
        estadoantiReboteBoton1 = CONFIRMACION;
      }


      break;


    case CONFIRMACION:
      if (mSegantiReboteBoton1 >= TIEMPO_antiReboteBoton1 && digitalRead(BOTON_1) == HIGH) {
        Serial.println("FalseAlarm");
        estadoantiReboteBoton1 = ESPERA;
      }
      if (mSegantiReboteBoton1 >= TIEMPO_antiReboteBoton1 && digitalRead(BOTON_1) == LOW) {
        Serial.println("Yendoaliberacion");
        estadoantiReboteBoton1 = LIBERACION;
      }


      break;


    case LIBERACION:
      if (digitalRead(BOTON_1) == HIGH) {
        flagBoton1 = 1;
        Serial.println("Liberado");
        estadoantiReboteBoton1 = ESPERA;
      }
      break;
  }
}
