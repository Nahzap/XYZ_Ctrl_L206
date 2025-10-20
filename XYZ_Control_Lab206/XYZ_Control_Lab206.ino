// =========================================================================
// --- CONFIGURACIÓN DE PINES ---
// =========================================================================

// Pines para los potenciómetros que controlan los motores
const byte potPinA = A0;
const byte potPinB = A1;

// Pines para el Driver L298N
// Motor A
const byte ENA = 3;  // Control de velocidad del Motor A (PWM)
const byte IN1 = 6;  // Control de dirección del Motor A
const byte IN2 = 7;  // Control de dirección del Motor A

// Motor B
const byte ENB = 11; // Control de velocidad del Motor B (PWM)
const byte IN3 = 12; // Control de dirección del Motor B
const byte IN4 = 13; // Control de dirección del Motor B

// =========================================================================
// --- PARÁMETROS DE CONTROL ---
// =========================================================================

// Umbrales para la "zona muerta" del potenciómetro
// Entre 337 y 675 el motor estará detenido.
const int umbralBajo = 337;
const int umbralAlto = 675;

// La potencia máxima que se aplicará al motor (0-255). 128 es aprox. 50%.
const int POTENCIA_MAX = 128; 
// La potencia mínima para que el motor empiece a moverse. Ajusta si es necesario.
const int POTENCIA_MIN = 65;


void setup() {
  // Iniciar comunicación serial a una velocidad alta para no perder datos.
  Serial.begin(115200); 
  Serial.println("Iniciando control de motores...");

  // Configurar todos los pines de los motores como SALIDAS
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  // Asegurarse de que los motores estén detenidos al iniciar
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
  Serial.println("Sistema listo. Mueve los potenciómetros.");
}

void loop() {
  // 1. Leer el valor de cada potenciómetro (rango de 0 a 1023)
  int valorPotA = analogRead(potPinA);
  int valorPotB = analogRead(potPinB);

  // 2. Controlar cada motor y obtener la potencia calculada
  int potenciaA = controlarMotor(ENA, IN1, IN2, valorPotA, false); // false = no invertir giro
  int potenciaB = controlarMotor(ENB, IN3, IN4, valorPotB, false); // false = no invertir giro

  // 3. Imprimir las potencias en el monitor serial en una sola línea
  Serial.print("Potencia Motor A: ");
  Serial.print(potenciaA);
  Serial.print("  |  Potencia Motor B: ");
  Serial.println(potenciaB);
  
  // Pequeña pausa para no saturar el monitor serial y hacer la lectura más fácil
  delay(10); 
}

/**
 * @brief Controla un motor (velocidad y dirección) basado en el valor de un potenciómetro.
 * @param pE Pin Enable (PWM) del motor.
 * @param p1 Pin de dirección 1 del motor.
 * @param p2 Pin de dirección 2 del motor.
 * @param v Valor del potenciómetro (0-1023).
 * @param i Booleano para invertir la dirección del motor (true/false).
 * @return La potencia calculada (0-255) que se está aplicando al motor.
 */
int controlarMotor(byte pE, byte p1, byte p2, int v, bool i) {
  byte d1 = HIGH, d2 = LOW; // Dirección por defecto
  int potencia = 0;

  // Si se pide invertir, se cambian las señales de dirección
  if (i) {
    d1 = LOW;
    d2 = HIGH;
  }
  
  // --- Lógica de control con zona muerta ---
  
  // Zona BAJA (Giro en una dirección)
  if (v <= umbralBajo) {
    // Escala la potencia desde POTENCIA_MIN (en umbralBajo) hasta POTENCIA_MAX (en 0)
    potencia = map(v, umbralBajo, 0, POTENCIA_MIN, POTENCIA_MAX);
    
    digitalWrite(p1, d2); // Dirección invertida
    digitalWrite(p2, d1);
    analogWrite(pE, potencia);
  } 
  // Zona MUERTA (Motor detenido)
  else if (v > umbralBajo && v <= umbralAlto) {
    potencia = 0;
    analogWrite(pE, 0); // Detiene el motor
  } 
  // Zona ALTA (Giro en la otra dirección)
  else {
    // Escala la potencia desde POTENCIA_MIN (en umbralAlto) hasta POTENCIA_MAX (en 1023)
    potencia = map(v, umbralAlto, 1023, POTENCIA_MIN, POTENCIA_MAX);
    
    digitalWrite(p1, d1); // Dirección normal
    digitalWrite(p2, d2);
    analogWrite(pE, potencia);
  }
  
  // Devuelve la potencia calculada para poder imprimirla
  return potencia;
}