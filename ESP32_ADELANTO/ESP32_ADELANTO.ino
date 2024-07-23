#include <ESP32Servo.h>

#define MAX_SIGNAL 2000
#define MIN_SIGNAL 1000
#define MOTOR_PIN1 18 // GPIO 18
#define MOTOR_PIN2 19 // GPIO 19
#define POT_ANGLE_PIN 35    // GPIO 34, para medir el ángulo
//#define POT_REF_PIN 34      // GPIO 35, para la referencia de voltaje
const int CONSTANT_SPEED = 1150;
bool motor2Running = true;

const int FILTER_SIZE = 10;
float errorBuffer[FILTER_SIZE] = {0};

// Variables para la ecuación en diferencias del controlador
float previousU = 0;
float previousY = 0;
float currentU = 0;
float currentY = 0;

// Variables para control del tiempo de muestreo
unsigned long lastTime = 0;
const float sampleTime = 0.00105; // Tiempo de muestreo en segundos (1.05 ms)

// Variable para controlar el tiempo de establecimiento inicial
unsigned long startTime = 0;
const unsigned long settlingTime = 3000; // Tiempo de establecimiento en milisegundos (5 segundos)

Servo motor1;
Servo motor2;

float refVoltage = 1.6; // Valor inicial de referencia de voltaje
float ref = 1.6;


void setup() {
  Serial.begin(9600);
  Serial.println("CALIBRACIÓN DEL ESC...");
  Serial.println(" ");
  delay(1500);
  Serial.println("Empieza el programa..");
  delay(1000);
  Serial.println("Inicialización del ESC.");

  motor1.attach(MOTOR_PIN1);
  motor2.attach(MOTOR_PIN2);

  // Calibración de ambos motores con señal máxima
  Serial.print("Now writing maximum output: ("); Serial.print(MAX_SIGNAL); Serial.print(" us in this case)"); Serial.print("\n");
  Serial.println("Turn on power source, then wait 2 seconds and press any key.");
  motor1.writeMicroseconds(MAX_SIGNAL);
  motor2.writeMicroseconds(MAX_SIGNAL);

  // Esperar entrada del usuario
  while (!Serial.available());
  Serial.read();

  // Calibración de ambos motores con señal mínima
  Serial.println("\n");
  Serial.println("Sending minimum output: ("); Serial.print(MIN_SIGNAL); Serial.print(" us in this case)"); Serial.print("\n");
  motor1.writeMicroseconds(MIN_SIGNAL);
  motor2.writeMicroseconds(MIN_SIGNAL);

  // Pequeña demora para asegurar que la calibración se complete
  delay(2000);

  Serial.println("The ESCs are calibrated");
  Serial.println("----");
  Serial.println("Motor2 is set to a constant speed of 1150 us");
  Serial.println("Now, adjust the potentiometer to control motor1 speed.");
  Serial.println("Type 'S2' to stop motor2");
  Serial.println("Type 'SET X.X' to set reference voltage");

  // Configurar motor2 a velocidad constante
  motor2.writeMicroseconds(CONSTANT_SPEED);

  // Configurar motor1 a velocidad inicial del 6%
  motor1.writeMicroseconds(1060); // 6% de la velocidad

  // Inicializar el tiempo de inicio
  startTime = millis();
}

void loop() {
  unsigned long currentTime = millis();
  unsigned long elapsedTime = currentTime - lastTime;
  unsigned long runningTime = currentTime - startTime;

  // Verificar si ha pasado el tiempo de muestreo
  if (elapsedTime >= sampleTime * 1000) { // Convertir tiempo de muestreo a milisegundos
    lastTime = currentTime;

    if (Serial.available() > 0) {
      String input = Serial.readStringUntil('\n'); // Leer la entrada hasta un carácter de nueva línea
      input.trim(); // Eliminar cualquier espacio en blanco al inicio o al final

      if (input.equals("S2")) {
        motor2Running = false;
        motor2.writeMicroseconds(MIN_SIGNAL); // Detener motor2
        Serial.println("Motor2 stopped");
      } else if (input.startsWith("SET ")) {
        input.remove(0, 4); // Eliminar el comando 'SET '
        ref = input.toFloat(); // Convertir el valor a float
        Serial.print("Reference voltage set to: "); Serial.print(refVoltage); Serial.println(" V");
      }
    }
    refVoltage = ((ref)/(24.1379))+1.5498;
    // Leer valores de los potenciómetros
    int potAngleValue = analogRead(POT_ANGLE_PIN);
    //int potRefValue = analogRead(POT_REF_PIN);

    // Convertir los valores de los potenciómetros a voltaje
    float angleVoltage = potAngleValue * (3.3 / 4095.0);

    float error = refVoltage - angleVoltage;

    for (int i = FILTER_SIZE - 1; i > 0; i--) {
      errorBuffer[i] = errorBuffer[i - 1];
    }

    errorBuffer[0] = error;

    // Calcular la media móvil del error
    float filteredError = 0;
    for (int i = 0; i < FILTER_SIZE; i++) {
      filteredError += errorBuffer[i];
    }
    filteredError /= FILTER_SIZE;

    // Si ha pasado el tiempo de establecimiento, comenzar a ajustar la velocidad del motor
    if (runningTime >= settlingTime) {
      // Calcular la señal de control (u)
      currentU = filteredError; // Aquí puedes agregar cualquier transformación o ganancia si es necesario

      // Implementar la ecuación en diferencias
      currentY = 1.772 * currentU - 1.771 * previousU ;
      
      currentY = currentY + 0.87;
      // Actualizar los valores anteriores
      previousU = currentU;
      previousY = currentY;

      // Mapear la señal de control al rango de velocidad del motor1
      int mappedSpeed = map(currentY * 1000, 0, 3300, 1060, 1220); // Ajustar los límites si es necesario

      // Limitar la velocidad del motor a su rango máximo y mínimo
      if (mappedSpeed > 1220) {
        mappedSpeed = 1220;
      } else if (mappedSpeed < 1060) {
        mappedSpeed = 1060;
      }

      // Ajustar la velocidad del motor1
      motor1.writeMicroseconds(mappedSpeed);

      float SPEED = (mappedSpeed - 1000) / 10.0;

      
      //Serial.print(refVoltage +0.15);
      Serial.print(error);
      Serial.print("\n");
    } else {
      // Mostrar el tiempo restante para el ajuste
      //Serial.print("Stabilizing... Time remaining: "); Serial.print((settlingTime - runningTime) / 1000.0); Serial.println(" seconds");
    }

    if (motor2Running) {
      motor2.writeMicroseconds(CONSTANT_SPEED); // Mantener motor2 a velocidad constante
    }
  }
}