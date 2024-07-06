#define MOT_L_A 7
#define MOT_L_B 6
#define MOT_L_PWM 11
#define MOT_R_A 9
#define MOT_R_B 12
#define MOT_R_PWM 10

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