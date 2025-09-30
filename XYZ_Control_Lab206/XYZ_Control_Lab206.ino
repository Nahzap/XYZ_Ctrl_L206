#include <Wire.h>
#include <VL53L0X.h>
#include <PID_v1.h> //By Brett Beauregard <br3ttb@gmail.com> brettbeauregard.com

// --- Variable de control para la interrupción del temporizador ---
volatile bool pid_update_flag = false;

// =========================================================================
// --- CONFIGURACIÓN DEL ESTABILIZADOR PID (SINTONIZADO) ---
// =========================================================================
double KP = 0.1;
double KI = 0.2;
double KD = 0.01;
double setpointX, inputX, outputX;
double setpointY, inputY, outputY;
double setpointZ, inputZ, outputZ;
PID pidX(&inputX, &outputX, &setpointX, KP, KI, KD, DIRECT);
PID pidY(&inputY, &outputY, &setpointY, KP, KI, KD, DIRECT);
PID pidZ(&inputZ, &outputZ, &setpointZ, KP, KI, KD, DIRECT);

// =========================================================================
// --- CONFIGURACIÓN DE CORRECCIÓN LINEAL (y = mx + b) ---
// =========================================================================
struct CorrectionParams {
  float m;
  float b;
};
CorrectionParams correctionX = { 10.0, 0.0 };
CorrectionParams correctionY = { 10.0, 0.0 };
CorrectionParams correctionZ = { 10.0, 0.0 };

// =========================================================================
// --- CONFIGURACIÓN DE SENSORES Y MOTORES (Sin cambios) ---
// =========================================================================
const byte SENSOR_X_XSHUT_PIN = 2, SENSOR_Y_XSHUT_PIN = 4, SENSOR_Z_XSHUT_PIN = 5;
const uint8_t SENSOR_X_ADDRESS = 0x30, SENSOR_Y_ADDRESS = 0x31, SENSOR_Z_ADDRESS = 0x32;
VL53L0X sensorX, sensorY, sensorZ;
const byte ENA = 3, IN1 = 6, IN2 = 7, ENB = 11, IN3 = 12, IN4 = 13;
const byte potPinA = A0, potPinB = A1;
const int umbralBajo = 337, umbralAlto = 675;
const byte velocidadFija = 128;

// =========================================================================
// --- ¡NUEVO! RUTINA DE SERVICIO DE INTERRUPCIÓN (ISR) ---
// =========================================================================
// Esta función se ejecuta automáticamente cada vez que el Timer1 llega a la cuenta.
// Se ejecuta exactamente cada 100ms.
ISR(TIMER1_COMPA_vect) {
  pid_update_flag = true;  // Activa la bandera para que el loop() haga los cálculos
}

void setup() {
  Serial.begin(115200);
  Serial.println(F("Iniciando sistema con Timer1 y PID..."));

  // --- Inicialización de Sensores y Motores (sin cambios) ---
  Wire.begin();
  pinMode(SENSOR_X_XSHUT_PIN, OUTPUT);
  pinMode(SENSOR_Y_XSHUT_PIN, OUTPUT);
  pinMode(SENSOR_Z_XSHUT_PIN, OUTPUT);
  digitalWrite(SENSOR_X_XSHUT_PIN, LOW);
  digitalWrite(SENSOR_Y_XSHUT_PIN, LOW);
  digitalWrite(SENSOR_Z_XSHUT_PIN, LOW);
  delay(100);
  initializeSensor(sensorX, SENSOR_X_XSHUT_PIN, SENSOR_X_ADDRESS, "Sensor X");
  initializeSensor(sensorY, SENSOR_Y_XSHUT_PIN, SENSOR_Y_ADDRESS, "Sensor Y");
  initializeSensor(sensorZ, SENSOR_Z_XSHUT_PIN, SENSOR_Z_ADDRESS, "Sensor Z");
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);

  delay(200);  // Esperar una lectura válida
  inputX = (float)sensorX.readRangeContinuousMillimeters() / 128.0;
  inputY = (float)sensorY.readRangeContinuousMillimeters() / 128.0;
  inputZ = (float)sensorZ.readRangeContinuousMillimeters() / 128.0;

  // --- Configurar los PIDs ---
  pidX.SetMode(AUTOMATIC);
  pidY.SetMode(AUTOMATIC);
  pidZ.SetMode(AUTOMATIC);
  pidX.SetOutputLimits(-255, 255);
  pidY.SetOutputLimits(-255, 255);
  pidZ.SetOutputLimits(-255, 255);

  // --- ¡NUEVO! CONFIGURACIÓN DEL TIMER1 PARA 100ms ---
  cli();  // Detener interrupciones
  // Configurar Timer1 en modo CTC (Clear Timer on Compare Match)
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = 0;
  // Frecuencia de interrupción = 10Hz (cada 100ms)
  // OCR1A = (Clock Speed / (Prescaler * Frecuencia)) - 1
  // OCR1A = (8,000,000 / (256 * 10)) - 1 = 3124
  OCR1A = 3124;
  // Activar modo CTC (WGM12) y prescaler de 256 (CS12)
  TCCR1B |= (1 << WGM12) | (1 << CS12);
  // Habilitar la interrupción por comparación del Timer1
  TIMSK1 |= (1 << OCIE1A);
  sei();  // Reactivar interrupciones

  Serial.println(F("\nSistema listo."));
}

void loop() {
  // El loop principal puede encargarse de tareas no críticas en el tiempo
  // --- 1. LECTURA DE POTENCIÓMETROS Y CONTROL DE MOTORES ---
  int valorPotA = analogRead(potPinA);
  int valorPotB = analogRead(potPinB);
  controlarMotor(ENA, IN1, IN2, valorPotA, false);
  controlarMotor(ENB, IN3, IN4, valorPotB, false);

  // --- 2. VERIFICAR SI ES HORA DE ACTUALIZAR EL PID ---
  if (pid_update_flag) {
    // --- Tareas que se ejecutan cada 100ms exactos ---

    // Leer sensores
    setpointX = (float)sensorX.readRangeContinuousMillimeters() / 128.0;
    setpointY = (float)sensorY.readRangeContinuousMillimeters() / 128.0;
    setpointZ = (float)sensorZ.readRangeContinuousMillimeters() / 128.0;

    // Calcular PIDs
    pidX.Compute();
    pidY.Compute();
    pidZ.Compute();
    inputX += outputX;
    inputY += outputY;
    inputZ += outputZ;

    // Aplicar corrección
    float finalX = aplicarCorreccion(inputX, correctionX);
    float finalY = aplicarCorreccion(inputY, correctionY);
    float finalZ = aplicarCorreccion(inputZ, correctionZ);

    // Imprimir datos
    Serial.print(F("Pots: A="));
    Serial.print(valorPotA);
    Serial.print(F(", B="));
    Serial.println(valorPotB);
    Serial.print(F(" X -> Crudo: "));
    printDistance(aplicarCorreccion(setpointX, correctionX));
    Serial.print(F(" | Filtrado: "));
    printDistance(finalX);
    Serial.println();
    Serial.print(F(" Y -> Crudo: "));
    printDistance(aplicarCorreccion(setpointY, correctionY));
    Serial.print(F(" | Filtrado: "));
    printDistance(finalY);
    Serial.println();
    Serial.print(F(" Z -> Crudo: "));
    printDistance(aplicarCorreccion(setpointZ, correctionZ));
    Serial.print(F(" | Filtrado: "));
    printDistance(finalZ);
    Serial.println();
    Serial.println(F("------------------------------------"));

    pid_update_flag = false;  // Resetear la bandera hasta la próxima interrupción
  }
}

// =========================================================================
// --- FUNCIONES AUXILIARES (sin cambios) ---
// =========================================================================
float aplicarCorreccion(float d, const CorrectionParams &p) {
  return (p.m * d) + p.b;
}
void printDistance(float d) {
  if (d < 0) {
    Serial.print(F("Error"));
    return;
  }
  int u = (int)d;
  int m = (int)fabs((d - u) * 10000);
  Serial.print(u);
  Serial.print(F(","));
  if (m < 1000) Serial.print(F("0"));
  if (m < 100) Serial.print(F("0"));
  if (m < 10) Serial.print(F("0"));
  Serial.print(m);
}
void controlarMotor(byte pE, byte p1, byte p2, int v, bool i) {
  byte d1 = HIGH, d2 = LOW;
  if (i) {
    d1 = LOW;
    d2 = HIGH;
  }
  if (v <= umbralBajo) {
    digitalWrite(p1, d2);
    digitalWrite(p2, d1);
    analogWrite(pE, velocidadFija);
  } else if (v > umbralBajo && v <= umbralAlto) {
    digitalWrite(p1, LOW);
    digitalWrite(p2, LOW);
    analogWrite(pE, 0);
  } else {
    digitalWrite(p1, d1);
    digitalWrite(p2, d2);
    analogWrite(pE, velocidadFija);
  }
}
void initializeSensor(VL53L0X &s, byte p, uint8_t a, const char *n) {
  digitalWrite(p, HIGH);
  delay(10);
  s.init();
  s.setAddress(a);
  s.setTimeout(500);
  s.writeReg(0x09, s.readReg(0x09) | 0x02);
  s.setMeasurementTimingBudget(200000);
  s.startContinuous();
  Serial.print(F("-> "));
  Serial.print(n);
  Serial.print(F(" inicializado en 0x"));
  Serial.println(a, HEX);
}