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

// Variables para la ecuación en diferencias del precompensador y del compensador
float error[2] = {0, 0}; // Entradas del compensador (error)
float integrado[2] = {0, 0}; // Salidas del compensador
float derivativo[2] = {0, 0};
float y_comp =0;


// Variables para control del tiempo de muestreo
unsigned long lastTime = 0;
const float sampleTime = 0.00105; // Tiempo de muestreo en segundos (1.05 ms)

// Variable para controlar el tiempo de establecimiento inicial
unsigned long startTime = 0;
const unsigned long settlingTime = 3000; // Tiempo de establecimiento en milisegundos (5 segundos)

Servo motor1;
Servo motor2;

float refVoltage = 1.5; // Valor inicial de referencia de voltaje
float ref = 1.5;

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
      if (input.startsWith("SET ")) {
        input.remove(0, 4); // Eliminar el comando 'SET '
        ref = input.toFloat(); // Convertir el valor a float
        Serial.print("Reference voltage set to: "); Serial.print(refVoltage); Serial.println(" V");
      }
    }

    refVoltage = ((ref)/(21.428))+1.51;

    // Leer valores de los potenciómetros
    int potAngleValue = analogRead(POT_ANGLE_PIN);
    //int potRefValue = 2047;

    // Convertir los valores de los potenciómetros a voltaje
    float angleVoltage = potAngleValue * (3.3 / 4095.0);
    //float refVoltage = 1.71;

    for (int i = FILTER_SIZE - 1; i > 0; i--) {
      errorBuffer[i] = errorBuffer[i - 1];
    }

    errorBuffer[0] = angleVoltage;

    // Calcular la media móvil del error
    float filteredError = 0;
    for (int i = 0; i < FILTER_SIZE; i++) {
      filteredError += errorBuffer[i];
    }
    filteredError /= FILTER_SIZE;

    // Si ha pasado el tiempo de establecimiento, comenzar a ajustar la velocidad del motor
    if (runningTime >= settlingTime) {
    
      // Calcular el error con realimentación unitaria
      float errores = refVoltage - filteredError;

      float Kp = 1.00496;
      float Ki = 1.61743;
      float Kd = 0.0005681;
      float Td = Kd/Kp;
      float N =100;
      // Descomentar Kd para probar el PID 
      
      // Compensador
      error[1] = error[0];
      error[0] = errores; // El error es la entrada del compensador

      float P = error[0] * Kp;
      float I = integrado[1] + (Ki*0.002*error[1]);
      float D = (Td/((N*0.002)+Td))*derivativo[1] + ((Kd*N)/((N*0.002)+Td))*( error[0] - error[1]);

      integrado[1]= I;
      derivativo[1]= D;

      float y_comp = P + I + D;

      float duty=  y_comp*0.058;
      float duty_motor=  (duty*1000)+1000;

      Serial.print("duty motor: "); Serial.print(duty*100); Serial.println("%");
      //Serial.print("duty motor conversion: "); Serial.print(duty_motor); 

      if(duty_motor > 1250){duty_motor=1250;}
      else if(duty_motor < 1000){duty_motor=1000;}

      // Ajustar la velocidad del motor1
      motor1.writeMicroseconds(duty_motor);
    } 

    if (motor2Running) {
      motor2.writeMicroseconds(CONSTANT_SPEED); // Mantener motor2 a velocidad constante
    }
  }
}