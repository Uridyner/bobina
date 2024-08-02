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

void setup() {
  pinMode(CNY1, INPUT);
  pinMode(CNY2, INPUT);

  Serial.begin(115200);

  setupMotores();
}

void loop() {
  uu();
}

void uu() {
  valorCNY1 = analogRead(CNY1);
  valorCNY2 = analogRead(CNY2);
  Serial.print("1: ");
  Serial.println(valorCNY1);
  Serial.print("2: ");
  Serial.println(valorCNY2);

}
