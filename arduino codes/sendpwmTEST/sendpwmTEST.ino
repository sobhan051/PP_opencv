// Define motor control pins (replace with your actual pins)
const int motor1Pin1 = 3;
const int motor1Pin2 = 4;
const int motor2Pin1 = 5;
const int motor2Pin2 = 6;
const int motorEnA = 9;
const int motorEnB = 10;

void setup() {
  Serial.begin(9600);
  // Set motor pins as outputs
  pinMode(motor1Pin1, OUTPUT);
  pinMode(motor1Pin2, OUTPUT);
  pinMode(motor2Pin1, OUTPUT);
  pinMode(motor2Pin2, OUTPUT);
  pinMode(motorEnA, OUTPUT);
  pinMode(motorEnB, OUTPUT);

}

void loop() {
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    char direction = command.charAt(0);
    int duration = command.substring(1).toInt();

    if (direction == 'R') {
      rotateRight(duration);
    } else if (direction == 'L') {
      rotateLeft(duration);
    }
  }
}

void rotateRight(int duration) {
  // Example: Control motors for right rotation
  digitalWrite(motor1Pin1, HIGH);
  digitalWrite(motor1Pin2, LOW);
  digitalWrite(motor2Pin1, LOW);
  digitalWrite(motor2Pin2, HIGH);
  analogWrite(motorEnA, 100);  // Set speed for Motor 1
  analogWrite(motorEnB, 100);  // Set speed for Motor 2
  delay(duration);
  stopMotors();
}

void rotateLeft(int duration) {
  // Example: Control motors for left rotation
  digitalWrite(motor1Pin1, LOW);
  digitalWrite(motor1Pin2, HIGH);
  digitalWrite(motor2Pin1, HIGH);
  digitalWrite(motor2Pin2, LOW);
  analogWrite(motorEnA, 100);  // Set speed for Motor 1
  analogWrite(motorEnB, 100);  // Set speed for Motor 2
  delay(duration);
  stopMotors();
}

void stopMotors() {
  // Stop all motors
  digitalWrite(motor1Pin1, LOW);
  digitalWrite(motor1Pin2, LOW);
  digitalWrite(motor2Pin1, LOW);
  digitalWrite(motor2Pin2, LOW);
  analogWrite(motorEnA, 0);  // Set speed for Motor 1
  analogWrite(motorEnB, 0);  // Set speed for Motor 2
}