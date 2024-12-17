//------------------------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------------------------
// 
//                                     Author : Sandun Meesara Nakandala
//                                     Date   : 2024-12-16
//
//------------------------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------------------------

#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

//Macros
#define run_speed 100
#define turn_speed 70
#define correction_speed 150
#define threshold_angle 0.25

//Pin Definition for Motor Controller
#define ENA 9 // PWM Pin for motor speed (Right Motor)
#define ENB 10 // PWM Pin for motor speed (Left Motor)
#define IN1 4  // Motor Right direction
#define IN2 5  // Motor Right direction
#define IN3 6  // Motor Left direction
#define IN4 7 // Motor Left direction

//Object Creations
Adafruit_MPU6050 mpu;

//---------------------------------Variables---------------------------------------------
float desiredYaw = 0.0f; // Desired yaw value when moving forward or backward
bool commands_given = false;


//---------------------------------Functions---------------------------------------------
// Motor control functions
void moveForward() {
  desiredYaw = getYaw();
  Serial.print("Desired Yaw: ");
  Serial.println(desiredYaw);
  analogWrite(ENA, run_speed); // Max speed for Motor A
  analogWrite(ENB, run_speed); // Max speed for Motor B
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

void moveBackward() {
  desiredYaw = getYaw();
  Serial.print("Desired Yaw: ");
  Serial.println(desiredYaw);
  analogWrite(ENA, run_speed); // Max speed for Motor A
  analogWrite(ENB, run_speed); // Max speed for Motor B
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}

void turnLeft() {
  analogWrite(ENA, turn_speed); // Max speed for Motor A
  analogWrite(ENB, turn_speed); // Max speed for Motor B
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  Serial.println("Turning left");
}

void turnRight() {
  analogWrite(ENA, turn_speed); // Max speed for Motor A
  analogWrite(ENB, turn_speed); // Max speed for Motor B
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  Serial.println("Turning right");
}

void stopMotors() {
  analogWrite(ENA, 0); // Stop Motor A
  analogWrite(ENB, 0); // Stop Motor B
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  Serial.println("Motors stopped");
}

// Function to read yaw (heading) from MPU6050
float getYaw() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // Calculate yaw from gyro data
  static float yaw = 0; // Persistent yaw
  static unsigned long lastTime = 0;
  unsigned long currentTime = millis();
  float dt = (currentTime - lastTime) / 1000.0;
  lastTime = currentTime;

  yaw += g.gyro.z * dt; // Integrate z-axis gyro rate
  return yaw;
}

// Direction correction function
void correctDirection(float currentYaw) {
  if(commands_given == true){
    float yawError = desiredYaw - currentYaw; // Calculate error between desired and current yaw
    // Normalize yaw error to be between -180 and 180 degrees
    if (yawError > 180) {
      yawError -= 360;
    } else if (yawError < -180) {
      yawError += 360;
    }
    /*Serial.print("desiredYaw: ");
    Serial.print(desiredYaw);
    Serial.print(" yawError: ");
    Serial.println(yawError);*/

    // If yaw error is too large, adjust motors to correct the direction
    if (abs(yawError) > threshold_angle) { // Threshold of x degrees for correction
      Serial.print("yaw error: ");
      Serial.println(yawError);
      Serial.println("Angle Correction Required!");
      if (yawError > 0) {
        // Rotate right if yawError is positive
        analogWrite(ENA, correction_speed);
        analogWrite(ENB, correction_speed);
        digitalWrite(IN1, HIGH);
        digitalWrite(IN2, LOW);
        digitalWrite(IN3, LOW);
        digitalWrite(IN4, HIGH);
        Serial.println("To correction Rotate right!");
      } else {
        // Rotate left if yawError is negative
        analogWrite(ENA, correction_speed);
        analogWrite(ENB, correction_speed);
        digitalWrite(IN1, LOW);
        digitalWrite(IN2, HIGH);
        digitalWrite(IN3, HIGH);
        digitalWrite(IN4, LOW);
        Serial.println("To correction Rotate left!");
      }
    } else {
      // No correction needed, move straight
      analogWrite(ENA, run_speed);
      analogWrite(ENB, run_speed);
      digitalWrite(IN1, HIGH);
      digitalWrite(IN2, LOW);
      digitalWrite(IN3, HIGH);
      digitalWrite(IN4, LOW);
    }
  }
}


//------------------------------------------------Setup and Loop Functions--------------------------------------
void setup() {
  Serial.begin(9600);
    while (!Serial)
      delay(10); // Wait for Serial Monitor

  // Initialize motor pins
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  // Initialize MPU6050
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }

  //Set MPU6050 data range and filter bandwidth
  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_94_HZ);

}

void loop() {
  // Check for incoming commands
  if (Serial.available() > 0) {
    //commands_given = false;
    char command = Serial.read();
    if (command == '1') {
      moveForward();
      commands_given = true;
    } else if (command == '2') {
      moveBackward();
      commands_given = true;
    } else if (command == '3') {
      turnLeft();
      commands_given = false;
    } else if (command == '4') {
      turnRight();
      commands_given = false;
    } else if (command == '5') {
      stopMotors();
      commands_given = false;
    }
  }

  // Get filtered orientation
  float yaw = getYaw();

  //call function to correct direction if it's going forward or backward
  correctDirection(yaw);

}

