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

#define ESPERA 0
#define CONFIRMACION 1
#define LIBERACION 2
#define TIEMPO_antiReboteBoton1 15
int estadoantiReboteBoton1 = 0;
int mSegantiReboteBoton1 = 0;
int flagBoton1 = 0;


void setup() {
  pinMode(BOTON_1, INPUT_PULLUP);
  pinMode(BOTON_2, INPUT);

  pinMode(LED_V, OUTPUT);
  pinMode(LED_R1, OUTPUT);
  pinMode(LED_R2, OUTPUT);

  Timer1.initialize(1000);
  Timer1.attachInterrupt(callBack);
  Serial.begin(9600);
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
