// ─── Librerías necesarias ──────────────────────────────────────
#include <Servo.h>                   // Control de servomotores
#include <ESP8266WiFi.h>             // Conexión WiFi para ESP8266
#include <BlynkSimpleEsp8266.h>      // Blynk para ESP8266

// ─── Credenciales de Blynk ─────────────────────────────────────
#define BLYNK_TEMPLATE_ID "TMPL2cEf0Ku9M"
#define BLYNK_TEMPLATE_NAME "CHASKY BOT"
#define BLYNK_AUTH_TOKEN "GWX26g587j2j1hVMc4_CiB6yrvddKUFe"

char ssid[] = "CELINA";              // Nombre de red WiFi
char pass[] = "19712344";            // Contraseña de red

// ─── Definición de pines ───────────────────────────────────────
const int pinBrazo = 0;              // Servo brazo (GPIO0 → D3)
const int pinPlataforma = 16;        // Servo plataforma (GPIO16 → D0)
const int trigPin = 10;              // Trig del HC-SR04 (SD3)
const int echoPin = 9;               // Echo del HC-SR04 (SD2)

const int sensorIzq = 5;             // Sensor IR izquierdo (D1)
const int sensorDer = 4;             // Sensor IR derecho (D2)

const int IN1 = 14;                  // L298N motor izquierdo (D5)
const int IN2 = 12;                  // L298N motor izquierdo (D6)
const int IN3 = 13;                  // L298N motor derecho (D7)
const int IN4 = 15;                  // L298N motor derecho (D8)

const int ledIntegrado = 2;          // LED azul del ESP8266 (GPIO2 → D4)

// ─── Variables de control ──────────────────────────────────────
int velocidad = 120;                 // Velocidad PWM para los motores
bool motoresEncendidos = true;       // Control desde Blynk para habilitar/deshabilitar motores

// Ángulos de servos configurables desde sliders en Blynk
int inicioBrazo = 90, finBrazo = 0;
int inicioPlataforma = 90, finPlataforma = 0;

bool notificacionEnviada = false;    // Bandera para evitar notificaciones duplicadas

// ─── Objetos para servomotores ─────────────────────────────────
Servo brazo, plataforma;

// ─── Configuración inicial ─────────────────────────────────────
void setup() {
  Serial.begin(9600);                       // Habilita consola serial
  Blynk.begin(BLYNK_AUTH_TOKEN, ssid, pass); // Conecta el ESP8266 a WiFi + Blynk

  // Configura los pines como entrada o salida según su función
  pinMode(sensorIzq, INPUT);
  pinMode(sensorDer, INPUT);
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  pinMode(ledIntegrado, OUTPUT); digitalWrite(ledIntegrado, HIGH); // Apagado al inicio
  pinMode(trigPin, OUTPUT); pinMode(echoPin, INPUT);               // Sensor ultrasónico

  // Conecta los servos y los posiciona en su ángulo inicial
  brazo.attach(pinBrazo); plataforma.attach(pinPlataforma);
  brazo.write(inicioBrazo); plataforma.write(inicioPlataforma);

  detener();  // Motores apagados por seguridad al arrancar
}

// ─── Widgets de Blynk ──────────────────────────────────────────
BLYNK_WRITE(V0) { motoresEncendidos = param.asInt(); }      // Botón ON/OFF para motores
BLYNK_WRITE(V1) { velocidad = param.asInt(); }              // Slider para controlar velocidad
BLYNK_WRITE(V3) { digitalWrite(ledIntegrado, param.asInt() ? LOW : HIGH); } // Botón para LED integrado

// Sliders para configurar ángulos de inicio y final de cada servo
BLYNK_WRITE(V4) { inicioBrazo = param.asInt(); }
BLYNK_WRITE(V5) { finBrazo = param.asInt(); }
BLYNK_WRITE(V6) { inicioPlataforma = param.asInt(); }
BLYNK_WRITE(V7) { finPlataforma = param.asInt(); }

// ─── Lógica principal del robot ────────────────────────────────
void loop() {
  Blynk.run(); // Mantiene conexión y sincroniza datos con Blynk

  if (!motoresEncendidos) {
    detener();
    return;  // Si el botón está en OFF, no hace nada más
  }

  // Lectura de sensores infrarrojos y distancia
  int izq = digitalRead(sensorIzq);
  int der = digitalRead(sensorDer);
  long distancia = distanciaCM();  // Lee distancia desde el HC-SR04
  String estado = "";

  // ── EVENTO 1: obstáculo cercano, pero NO sobre línea negra ──
  if (distancia <= 5 && !(izq == HIGH && der == HIGH)) {
    estado = "OBSTÁCULO DETECTADO - FUERA DE LÍNEA";
    if (!notificacionEnviada) {
      Blynk.logEvent("obstaculo_detectado", "¡Objeto a 5 cm y CHASKY BOT fuera de la línea!");
      notificacionEnviada = true;  // Evita spam
    }
    avanzar(); // Opcional: puede quedarse detenido si prefieres
  }

  // ── EVENTO 2: ambos sensores detectan línea negra ───────────
  else if (izq == HIGH && der == HIGH) {
    detener();
    estado = "EN NEGRO - INICIA RUTINA";

    brazo.write(inicioBrazo);
    plataforma.write(inicioPlataforma); delay(300);     // Posición base
    brazo.write(finBrazo);
    plataforma.write(finPlataforma); delay(500);        // Acción

    avanzar(); delay(300);                              // Avance inicial breve

    while (distanciaCM() > 2) {                          // Sigue avanzando hasta 2cm
      avanzar(); delay(50);
    }

    detener();
    brazo.write(inicioBrazo); plataforma.write(inicioPlataforma); delay(300); // Reposo
    girar360(); avanzar();
    notificacionEnviada = false; // Reset de bandera
  }

  // ── MOVIMIENTOS NORMALES: Seguimiento de línea ──────────────
  else if (izq == LOW && der == HIGH) {
    girarIzquierda(); estado = "GIRO IZQUIERDA";
    notificacionEnviada = false;
  } else if (izq == HIGH && der == LOW) {
    girarDerecha(); estado = "GIRO DERECHA";
    notificacionEnviada = false;
  } else {
    avanzar(); estado = "AVANZANDO";
    notificacionEnviada = false;
  }

  Blynk.virtualWrite(V2, estado);  // Actualiza texto en el label
  delay(100);                      // Pequeño delay para estabilidad
}

// ─── Funciones de movimiento con PWM ──────────────────────────
void avanzar() {
  analogWrite(IN1, 0); analogWrite(IN2, velocidad);
  analogWrite(IN3, velocidad); analogWrite(IN4, 0);
}

void girarIzquierda() {
  analogWrite(IN1, 0); analogWrite(IN2, velocidad);
  analogWrite(IN3, 0); analogWrite(IN4, velocidad);
}

void girarDerecha() {
  analogWrite(IN1, velocidad); analogWrite(IN2, 0);
  analogWrite(IN3, velocidad); analogWrite(IN4, 0);
}

void detener() {
  analogWrite(IN1, 0); analogWrite(IN2, 0);
  analogWrite(IN3, 0); analogWrite(IN4, 0);
  brazo.write(inicioBrazo);
  plataforma.write(inicioPlataforma);
}

void girar360() {
  girarDerecha(); delay(1200); detener();  // Aproximadamente 360° (ajustable)
}

// ─── Sensor ultrasónico: mide la distancia en centímetros ─────
long distanciaCM() {
  digitalWrite(trigPin, LOW); delayMicroseconds(2);
  digitalWrite(trigPin, HIGH); delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duracion = pulseIn(echoPin, HIGH, 20000);  // Espera máxima: 20ms
  long distancia = duracion * 0.034 / 2;
  Serial.println("Distancia: " + String(distancia) + " cm");
  return distancia;
}