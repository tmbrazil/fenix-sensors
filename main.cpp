#include <Wire.h>
#include <Adafruit_MPU6050.h> // Biblioteca para MPU6050
#include <DHT.h>              // Biblioteca para sensor de temperatura/humidade

// Definições de pinos
#define HALL_SENSOR_PIN A0    // Sensor Hall (rotação das rodas)
#define OBSTACLE_PIN 2        // KY-032 (obstáculos)
#define INFRARED_PIN 3        // Sensor infravermelho
#define TILT_PIN 4            // KY-020 (inclinação)
#define VIBRATION_PIN 5       // SW-420 (vibração)
#define LIGHT_PIN A1          // Sensor de luz (LDR)
#define DHTPIN 6              // Pino do DHT11/DHT22
#define JOYSTICK_X A2         // Eixo X do joystick
#define JOYSTICK_Y A3         // Eixo Y do joystick
#define MOTOR_A1 7            // Controle do motor A (H-bridge)
#define MOTOR_A2 8
#define MOTOR_B1 9
#define MOTOR_B2 10

// Inicialização de sensores
Adafruit_MPU6050 mpu;
DHT dht(DHTPIN, DHT22); // Altere para DHT11 se necessário

void setup() {
  Serial.begin(9600);
  
  // Inicializa MPU6050
  if (!mpu.begin()) {
    Serial.println("Falha no MPU6050!");
    while (1);
  }

  // Configuração dos pinos
  pinMode(OBSTACLE_PIN, INPUT);
  pinMode(INFRARED_PIN, INPUT);
  pinMode(TILT_PIN, INPUT);
  pinMode(VIBRATION_PIN, INPUT);
  pinMode(MOTOR_A1, OUTPUT);
  pinMode(MOTOR_A2, OUTPUT);
  pinMode(MOTOR_B1, OUTPUT);
  pinMode(MOTOR_B2, OUTPUT);

  dht.begin();
}

void loop() {
  // Leitura de todos os sensores
  readMPU6050();
  readHallSensor();
  readObstacleSensors();
  readTiltVibration();
  readEnvironment();
  readJoystick();

  // Lógica de controle do rover
  if (detectObstacle()) {
    avoidObstacle();
  } else {
    moveRover();
  }

  delay(100); // Ajuste conforme necessário
}

// Funções de leitura de sensores
void readMPU6050() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  Serial.print("Aceleração X: "); Serial.println(a.acceleration.x);
  Serial.print("Giro Y: "); Serial.println(g.gyro.y);
}

void readHallSensor() {
  int hallValue = analogRead(HALL_SENSOR_PIN);
  Serial.print("Rotação: "); Serial.println(hallValue);
}

void readObstacleSensors() {
  bool obstacle = digitalRead(OBSTACLE_PIN);
  bool infrared = digitalRead(INFRARED_PIN);
  Serial.print("Obstáculo: "); Serial.println(obstacle);
  Serial.print("Infravermelho: "); Serial.println(infrared);
}

void readTiltVibration() {
  bool tilt = digitalRead(TILT_PIN);
  bool vibration = digitalRead(VIBRATION_PIN);
  Serial.print("Inclinação: "); Serial.println(tilt);
  Serial.print("Vibração: "); Serial.println(vibration);
}

void readEnvironment() {
  float humidity = dht.readHumidity();
  float temperature = dht.readTemperature();
  int light = analogRead(LIGHT_PIN);
  Serial.print("Temp: "); Serial.print(temperature);
  Serial.print(" Umidade: "); Serial.print(humidity);
  Serial.print(" Luz: "); Serial.println(light);
}

void readJoystick() {
  int x = analogRead(JOYSTICK_X);
  int y = analogRead(JOYSTICK_Y);
  // Mapear valores para controle de movimento
}

// Funções de controle
bool detectObstacle() {
  return digitalRead(OBSTACLE_PIN) || digitalRead(INFRARED_PIN);
}

void avoidObstacle() {
  // Exemplo: virar para a direita
  digitalWrite(MOTOR_A1, HIGH);
  digitalWrite(MOTOR_A2, LOW);
  digitalWrite(MOTOR_B1, LOW);
  digitalWrite(MOTOR_B2, HIGH);
  delay(500); // Tempo de manobra
}

void moveRover() {
  // Exemplo: mover para frente
  digitalWrite(MOTOR_A1, HIGH);
  digitalWrite(MOTOR_A2, LOW);
  digitalWrite(MOTOR_B1, HIGH);
  digitalWrite(MOTOR_B2, LOW);
}