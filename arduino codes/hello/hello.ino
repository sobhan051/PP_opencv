// Define motor pins
const int motor1Pin1 = 3;  // IN1 on L298N (Motor 1)
const int motor1Pin2 = 4;  // IN2 on L298N (Motor 1)
const int motor2Pin1 = 5;  // IN3 on L298N (Motor 2)
const int motor2Pin2 = 6;  // IN4 on L298N (Motor 2)
const int enablePin1 = 9;  // ENA on L298N (PWM for Motor 1)
const int enablePin2 = 10; // ENB on L298N (PWM for Motor 2)

void setup() {
  // Set motor pins as outputs
  pinMode(motor1Pin1, OUTPUT);
  pinMode(motor1Pin2, OUTPUT);
  pinMode(motor2Pin1, OUTPUT);
  pinMode(motor2Pin2, OUTPUT);
  pinMode(enablePin1, OUTPUT);
  pinMode(enablePin2, OUTPUT);

  // Initialize serial communication
  Serial.begin(9600);

  // Set initial motor state (stop)
  stopMotors();
}

void loop() {
  if (Serial.available() > 0) {
    // Read the rotation duration from the serial port
    int duration = Serial.parseInt();

    // Rotate the robot car
    rotateRobot(duration);

    // Stop the motors after rotation
    stopMotors();
  }
}

void rotateRobot(int duration) {
  // Set motors to rotate (adjust pins for your setup)
  // Motor 1: Rotate clockwise
  digitalWrite(motor1Pin1, HIGH);
  digitalWrite(motor1Pin2, LOW);
  // Motor 2: Rotate counterclockwise
  digitalWrite(motor2Pin1, LOW);
  digitalWrite(motor2Pin2, HIGH);

  // Enable both motors with a fixed PWM value (e.g., 200)
  analogWrite(enablePin1, 100);  // Set speed for Motor 1
  analogWrite(enablePin2, 100);  // Set speed for Motor 2

  // Wait for the specified duration
  delay(duration);
}

void stopMotors() {
  // Stop both motors
  digitalWrite(motor1Pin1, LOW);
  digitalWrite(motor1Pin2, LOW);
  digitalWrite(motor2Pin1, LOW);
  digitalWrite(motor2Pin2, LOW);

  // Disable both motors
  analogWrite(enablePin1, 0);  // Set speed to 0 for Motor 1
  analogWrite(enablePin2, 0);  // Set speed to 0 for Motor 2
}