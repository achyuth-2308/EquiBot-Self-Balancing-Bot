#include <Wire.h>
#include <MPU6050.h>

// Motor A (Left Motor)
int MotorA1 = 16;
int MotorA2 = 17;
int enable1Pin = 4;

// Motor B (Right Motor)
int MotorB1 = 18;
int MotorB2 = 19;
int enable2Pin = 5;

// PWM properties
const int freq = 1000;
const int pwmChannel1 = 0;
const int pwmChannel2 = 1;
const int resolution = 8;
int dutyCycle = 255;

// MPU6050 object
MPU6050 mpu;

// PID constants (tuned for better stability)
float Kp = 35;   // Increase P for stronger correction
float Ki = 0.1;    // Small integral to reduce steady-state error
float Kd = 1;    // Increased D for damping

// Variables for PID
float setPoint = -0.5;  // Desired angle (upright position)
float input, output;
float previousError = 0;
float integral = 0;
float maxIntegral = 50;  // Prevent integral windup

// Timing
unsigned long lastTime;

// Complementary filter parameters
float alpha = 0.98;  // Complementary filter weight

void setup() {
    Serial.begin(115200);

    // Initialize MPU6050
    Wire.begin();
    mpu.initialize();
    if (!mpu.testConnection()) {
        Serial.println("MPU6050 connection failed!");
        while (1);
    } else {
        Serial.println("MPU6050 connected!");
    }

    // Configure motor pins
    pinMode(MotorA1, OUTPUT);
    pinMode(MotorA2, OUTPUT);
    pinMode(MotorB1, OUTPUT);
    pinMode(MotorB2, OUTPUT);

    // Setup PWM
    ledcSetup(pwmChannel1, freq, resolution);
    ledcAttachPin(enable1Pin, pwmChannel1);
    ledcSetup(pwmChannel2, freq, resolution);
    ledcAttachPin(enable2Pin, pwmChannel2);

    lastTime = millis();
}

void loop() {
    // Read angle from MPU6050
    input = getTiltAngle();

    // Apply PID control
    float error = setPoint - input;
    unsigned long now = millis();
    float elapsedTime = (now - lastTime) / 1000.0;  // Time in seconds

    // Compute PID terms
    integral += error * elapsedTime;
    integral = constrain(integral, -maxIntegral, maxIntegral);  // Prevent integral windup

    float derivative = (error - previousError) / elapsedTime;
    output = Kp * error + Ki * integral + Kd * derivative;

    // Constrain output to avoid excessive speed
    output = constrain(output, -300, 300);  

    // Adjust motor speed
    setMotorSpeed(output);

    // Debugging information
    Serial.print("Angle: ");
    Serial.print(input);
    Serial.print(" | PID Output: ");
    Serial.println(output);

    previousError = error;
    lastTime = now;
    delay(10);  // Reduce delay for more frequent updates
}

// Complementary Filter for Smoother Angle Calculation
float getTiltAngle() {
    int16_t ax, ay, az, gx, gy, gz;
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    // Convert raw accelerometer data to 'g' force
    float accelAngle = atan2((float)ay / 16384.0, (float)az / 16384.0) * 180 / PI;

    // Convert gyroscope data to degrees per second
    float gyroRate = (float)gx / 131.0;  // 131 LSB/(deg/s)

    // Compute new angle using complementary filter
    static float angle = 0;
    float dt = (millis() - lastTime) / 1000.0;
    angle = alpha * (angle + gyroRate * dt) + (1 - alpha) * accelAngle;

    return angle;
}

// Adjust Motor Speed Smoothly
void setMotorSpeed(float pidOutput) {  
    int pwm = constrain(abs(pidOutput), 130, 255);
    Serial.print("SPEED : ");
    Serial.print(pwm);
    // Move forward with adjusted speed
    if (pidOutput > 0) {
            digitalWrite(MotorA1, LOW);
            digitalWrite(MotorA2, HIGH);
            digitalWrite(MotorB1, LOW);
            digitalWrite(MotorB2, HIGH); 
        } 
    else {
            digitalWrite(MotorA1, HIGH);
            digitalWrite(MotorA2, LOW);
            digitalWrite(MotorB1, HIGH);
            digitalWrite(MotorB2, LOW);
        }
        ledcWrite(pwmChannel1, pwm);
        ledcWrite(pwmChannel2, pwm);  
}
