#define BLYNK_TEMPLATE_ID "TMPL23PaTRcqS"
#define BLYNK_TEMPLATE_NAME "CHASKY BOTCopy"
#define BLYNK_AUTH_TOKEN "xM_bNU3QgirPS18ypoPHMeOI5SLPWKsP"

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <ESP8266WiFi.h>
#include <BlynkSimpleEsp8266.h>

char ssid[] = "CELINA";
char pass[] = "19712344";

bool robotActivo = false;

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// Pines de sensores
const int sensorIzquierdo = 0;  // D3 - GPIO0
const int sensorDerecho   = 2;  // D4 - GPIO2

// Pines para motores (L298N)
const int IN1 = 14;  // D5
const int IN2 = 12;  // D6
const int IN3 = 13;  // D7
const int IN4 = 15;  // D8

// Canales del PCA9685
const int S_brazoIZQ     = 0;
const int S_brazoDER     = 4;
const int S_platformIZQ  = 8;
const int S_platformDER  = 12;

#define SERVOMIN 102
#define SERVOMAX 512

int velocidad = 120;

// Posiciones actuales de los servos
int posBrazoIZQ = 0;
int posBrazoDER = 180;
int posPlatIZQ  = 25;
int posPlatDER  = 25;

bool yaRecogio = false;

BLYNK_WRITE(V0) {
  int estado = param.asInt();
  robotActivo = (estado == 1);
  if (!robotActivo) {
    detener();
    Serial.println("Robot apagado desde Blynk");
  } else {
    Serial.println("Robot encendido desde Blynk");
  }
}

BLYNK_WRITE(V1) {
  velocidad = param.asInt();  // Asigna el valor recibido al control de velocidad
  Serial.print("Velocidad actual: ");
  Serial.println(velocidad);
}

void setup() {
  pinMode(sensorIzquierdo, INPUT);
  pinMode(sensorDerecho, INPUT);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  Wire.begin(4, 5);  // SDA = D2, SCL = D1
  pwm.begin();
  pwm.setPWMFreq(50);

  Serial.begin(9600);
  Serial.println("Sistema iniciado.");

  Blynk.begin(BLYNK_AUTH_TOKEN, ssid, pass);

  moverServosSimultaneamente(
    posBrazoIZQ, 0, S_brazoIZQ, 0, 180,
    posBrazoDER, 180, S_brazoDER, 0, 180,
    posPlatIZQ,  0, S_platformIZQ, 0, 180,
    posPlatDER,  180, S_platformDER, 0, 180,
    10
  );

  detener();
}

void loop() {
  Blynk.run();

  if (!robotActivo) return;

  int estadoIzquierdo = digitalRead(sensorIzquierdo);
  int estadoDerecho   = digitalRead(sensorDerecho);

  if (estadoIzquierdo == HIGH && estadoDerecho == HIGH && !yaRecogio) {
    detener();
    Serial.println("NEGRO - PUNTO FINAL - ACTIVAR SERVOS");

    moverServosSimultaneamente(
      posBrazoIZQ, 180, S_brazoIZQ, 0, 180,
      posBrazoDER, 0,   S_brazoDER, 0, 180,
      posPlatIZQ,  180, S_platformIZQ, 0, 180,
      posPlatDER,  0,   S_platformDER, 0, 180,
      10
    );

    delay(1000);

    avanzar();
    delay(1000);
    detener();

    retroceder();
    delay(100);
    detener();

    moverServosSimultaneamente(
      posBrazoIZQ, 0,   S_brazoIZQ, 0, 180,
      posBrazoDER, 180, S_brazoDER, 0, 180,
      posPlatIZQ,  0,   S_platformIZQ, 0, 180,
      posPlatDER,  180, S_platformDER, 0, 180,
      10
    );

    delay(500);

    girarDerecha();
    delay(1400);
    detener();

    delay(300);

    avanzar();
    delay(1000);
    detener();

    yaRecogio = true;
  }

  else if (estadoIzquierdo == LOW && estadoDerecho == HIGH) {
    girarIzquierda();
  }

  else if (estadoIzquierdo == HIGH && estadoDerecho == LOW) {
    girarDerecha();
  }

  else if (estadoIzquierdo == LOW && estadoDerecho == LOW) {
    avanzar();
  }

  delay(100);
}

// === Movimiento de motores ===

void avanzar() {
  analogWrite(IN1, 0);
  analogWrite(IN2, velocidad);
  analogWrite(IN3, velocidad);
  analogWrite(IN4, 0);
  actualizarEstadoRobot("Avanzando");
}

void retroceder() {
  analogWrite(IN1, velocidad);
  analogWrite(IN2, 0);
  analogWrite(IN3, 0);
  analogWrite(IN4, velocidad);
  actualizarEstadoRobot("Retrocediendo");
}

void girarIzquierda() {
  analogWrite(IN1, 0);
  analogWrite(IN2, velocidad);
  analogWrite(IN3, 0);
  analogWrite(IN4, velocidad);
  actualizarEstadoRobot("Girando Izquierda");
}

void girarDerecha() {
  analogWrite(IN1, velocidad);
  analogWrite(IN2, 0);
  analogWrite(IN3, velocidad);
  analogWrite(IN4, 0);
  actualizarEstadoRobot("Girando Derecha");
}

void detener() {
  analogWrite(IN1, 0);
  analogWrite(IN2, 0);
  analogWrite(IN3, 0);
  analogWrite(IN4, 0);
  actualizarEstadoRobot("Detenido");
}


// === Movimiento suave y simultáneo de servos ===

void moverServosSimultaneamente(
  int &pos1, int objetivo1, int canal1, int min1, int max1,
  int &pos2, int objetivo2, int canal2, int min2, int max2,
  int &pos3, int objetivo3, int canal3, int min3, int max3,
  int &pos4, int objetivo4, int canal4, int min4, int max4,
  int delayPaso
) {
  objetivo1 = constrain(objetivo1, min1, max1);
  objetivo2 = constrain(objetivo2, min2, max2);
  objetivo3 = constrain(objetivo3, min3, max3);
  objetivo4 = constrain(objetivo4, min4, max4);

  int pasos = max({
    abs(objetivo1 - pos1),
    abs(objetivo2 - pos2),
    abs(objetivo3 - pos3),
    abs(objetivo4 - pos4)
  });

  for (int i = 0; i <= pasos; i++) {
    if (pos1 != objetivo1) {
      pos1 += (pos1 < objetivo1) ? 1 : -1;
      pwm.setPWM(canal1, 0, anguloToPulse(pos1));
    }
    if (pos2 != objetivo2) {
      pos2 += (pos2 < objetivo2) ? 1 : -1;
      pwm.setPWM(canal2, 0, anguloToPulse(pos2));
    }
    if (pos3 != objetivo3) {
      pos3 += (pos3 < objetivo3) ? 1 : -1;
      pwm.setPWM(canal3, 0, anguloToPulse(pos3));
    }
    if (pos4 != objetivo4) {
      pos4 += (pos4 < objetivo4) ? 1 : -1;
      pwm.setPWM(canal4, 0, anguloToPulse(pos4));
    }
    delay(delayPaso);
  }
}

uint16_t anguloToPulse(int angulo) {
  return map(angulo, 0, 180, SERVOMIN, SERVOMAX);
}
void actualizarEstadoRobot(String estado) {
  Serial.println("Estado: " + estado);
  Blynk.virtualWrite(V2, estado);
}



