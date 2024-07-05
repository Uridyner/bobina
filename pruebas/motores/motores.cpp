#define MOT_L_A -1
#define MOT_L_B -1
#define MOT_L_PWM -1
#define MOT_R_A -1
#define MOT_R_B -1
#define MOT_R_PWM -1

void adelante() {
    Serial.println("Adelante");
    digitalWrite(MOT_L_A, HIGH);
    digitalWrite(MOT_L_B, FALSE);
    digitalWrite(MOT_R_A, HIGH);
    digitalWrite(MOT_R_B, FALSE);
}

void atras() {
    Serial.println("Atras");
    digitalWrite(MOT_L_A, FALSE);
    digitalWrite(MOT_L_B, HIGH);
    digitalWrite(MOT_R_A, FALSE);
    digitalWrite(MOT_R_B, HIGH);
}

void izquierda() {
    Serial.println("Izquierda");
    digitalWrite(MOT_L_A, HIGH);
    digitalWrite(MOT_L_B, FALSE);
    digitalWrite(MOT_R_A, FALSE);
    digitalWrite(MOT_R_B, HIGH);
}

void derecha() {
    Serial.println("Derecha");
    digitalWrite(MOT_L_A, FALSE);
    digitalWrite(MOT_L_B, HIGH);
    digitalWrite(MOT_R_A, HIGH);
    digitalWrite(MOT_R_B, FALSE);
}

void parada() {
    Serial.println("Parada");
    digitalWrite(MOT_L_A, FALSE);
    digitalWrite(MOT_L_B, FALSE);
    digitalWrite(MOT_R_A, FALSE);
    digitalWrite(MOT_R_B, FALSE);
}

void frenar() {
    Serial.println("Frenado");
    digitalWrite(MOT_L_A, HIGH);
    digitalWrite(MOT_L_B, HIGH);
    digitalWrite(MOT_R_A, HIGH);
    digitalWrite(MOT_R_B, HIGH);
}

void setup() {
    Serial.begin(115200);
    
    Serial.println("Inicializando pines...");
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
    Serial.println("Pines inicializados!");
    
    Serial.println("Poniendo dirección inicial en los motores");
    parada();
    Serial.println("Puesta la dirección inicial en los motores!");
    
    Serial.println("Poniendo velocidad en los motores");
    analogWrite(MOT_L_PWM, 255);
    analogWrite(MOT_R_PWM, 255);
    Serial.println("Velocidad de los motores establecidas!");
    
    Serial.println("Comenzando loop en 2 segundos...");
    delay(2000);
}

void loop() {
    adelante();
    delay(1000);
    parada();
    delay(500);
    
    atras();
    delay(1000);
    parada();
    delay(500);
    
    izquierda();
    delay(1000);
    parada();
    delay(500);
    
    derecha();
    delay(1000);
    parada();
    delay(500);
}
