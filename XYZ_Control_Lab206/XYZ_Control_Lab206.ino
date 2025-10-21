// =========================================================================
// --- CONFIGURACIÓN DE PINES Y PARÁMETROS ---
// =========================================================================
const byte potPinA = A0;
const byte potPinB = A1;
const byte sensorPin1 = A2;
const byte sensorPin2 = A3;
const byte ENA = 3, IN1 = 6, IN2 = 7;
const byte ENB = 11, IN3 = 12, IN4 = 13;
const int umbralBajo = 337, umbralAlto = 675;
const int POTENCIA_MAX_MANUAL = 128, POTENCIA_MIN_MANUAL = 65;

// =========================================================================
// --- VARIABLES GLOBALES DEL SISTEMA ---
// =========================================================================
enum ControlMode { MANUAL, AUTO };
ControlMode currentMode = MANUAL;
int potenciaA = 0;
int potenciaB = 0;

// ¡NUEVO! Buffer para almacenar los comandos entrantes
char command_buffer[50];
byte buffer_pos = 0;

void setup() {
  Serial.begin(1000000); // Tasa de baudios corregida a 1,000,000

  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  setMotorPower(ENA, IN1, IN2, 0);
  setMotorPower(ENB, IN3, IN4, 0);
  
  // La cabecera se envía una sola vez para el plotter/análisis de datos
  Serial.println("PotenciaA,PotenciaB,Sensor1,Sensor2");
}

void loop() {
  // --- PASO 1: Revisar si hay nuevos comandos (lógica mejorada) ---
  checkSerialCommands();

  // --- PASO 2: Ejecutar el modo de control actual ---
  if (currentMode == MANUAL) {
    int valorPotA = analogRead(potPinA);
    int valorPotB = analogRead(potPinB);
    potenciaA = getManualPower(valorPotA);
    potenciaB = getManualPower(valorPotB);
    
    setMotorPower(ENA, IN1, IN2, potenciaA);
    setMotorPower(ENB, IN3, IN4, potenciaB);
  } else { // Modo AUTO
    // En modo AUTO, la potencia solo cambia cuando llega un comando nuevo.
    // Mientras tanto, se mantiene la última potencia asignada.
    setMotorPower(ENA, IN1, IN2, potenciaA);
    setMotorPower(ENB, IN3, IN4, potenciaB);
  }

  // --- PASO 3: Leer sensores y enviar datos (sin cambios) ---
  int valorSensor1 = analogRead(sensorPin1);
  int valorSensor2 = analogRead(sensorPin2);

  Serial.print(potenciaA);
  Serial.print(',');
  Serial.print(potenciaB);
  Serial.print(',');
  Serial.print(valorSensor1);
  Serial.print(',');
  Serial.println(valorSensor2);
}

// =========================================================================
// --- ¡NUEVA FUNCIÓN MEJORADA PARA PROCESAR COMANDOS! ---
// =========================================================================
void checkSerialCommands() {
  while (Serial.available() > 0) {
    char inChar = (char)Serial.read();

    // Si recibimos un salto de línea, el comando está completo
    if (inChar == '\n') {
      command_buffer[buffer_pos] = '\0'; // Terminar el string

      // --- Procesar el comando completo ---
      if (strcmp(command_buffer, "M") == 0 || strcmp(command_buffer, "m") == 0) {
        currentMode = MANUAL;
        // No es necesario imprimir mensajes de depuración que interfieran con los datos CSV
      } 
      else if (strncmp(command_buffer, "A,", 2) == 0 || strncmp(command_buffer, "a,", 2) == 0) {
        currentMode = AUTO;
        // Parsear las potencias del comando "A,potA,potB"
        char* token = strtok(command_buffer, ","); // "A"
        token = strtok(NULL, ",");                 // potA
        if (token != NULL) potenciaA = atoi(token);
        token = strtok(NULL, ",");                 // potB
        if (token != NULL) potenciaB = atoi(token);
      }
      
      // Reiniciar el buffer para el próximo comando
      buffer_pos = 0;
      
    } else {
      // Añadir el carácter al buffer si hay espacio
      if (buffer_pos < sizeof(command_buffer) - 1) {
        command_buffer[buffer_pos++] = inChar;
      }
    }
  }
}

// =========================================================================
// --- FUNCIONES AUXILIARES (Sin cambios) ---
// =========================================================================

void setMotorPower(byte pinE, byte pin1, byte pin2, int power) {
  power = constrain(power, -255, 255);
  if (power > 0) {
    digitalWrite(pin1, HIGH);
    digitalWrite(pin2, LOW);
    analogWrite(pinE, power);
  } else if (power < 0) {
    digitalWrite(pin1, LOW);
    digitalWrite(pin2, HIGH);
    analogWrite(pinE, -power);
  } else {
    digitalWrite(pin1, LOW);
    digitalWrite(pin2, LOW);
    analogWrite(pinE, 0);
  }
}

int getManualPower(int v) {
  int potencia = 0;
  if (v <= umbralBajo) {
    potencia = map(v, umbralBajo, 0, POTENCIA_MIN_MANUAL, POTENCIA_MAX_MANUAL);
    return -potencia;
  } 
  else if (v > umbralBajo && v <= umbralAlto) {
    return 0;
  } 
  else {
    potencia = map(v, umbralAlto, 1023, POTENCIA_MIN_MANUAL, POTENCIA_MAX_MANUAL);
    return potencia;
  }
}