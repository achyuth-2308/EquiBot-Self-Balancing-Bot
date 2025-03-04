# EquiBot: Self-Balancing-Bot 🤖ིྀ

A self-balancing bot using **ESP32**, **MPU6050**, **L293D Motor Driver** and a **PID controller** to maintain dynamic stability. This project implements an **inverted pendulum model** to keep the bot upright.

---

## 📌 Features
- **Real-time tilt correction** using MPU6050 sensor data.
- **PID control algorithm** for precise motor adjustments.
- **ESP32-based** implementation with efficient processing.
- **DC motor control** via motor driver for smooth balancing.
- **Expandable** for Bluetooth/WiFi remote control and advanced tuning.

---

## 🛠 Components Used

| Component      | Description |
|---------------|------------|
| **ESP32**     | Microcontroller for processing sensor data |
| **MPU6050**   | 6-axis motion sensor (accelerometer + gyroscope) |
| **Motor Driver** | Controls the direction and speed of motors |
| **DC Motors** | Provides motion and balance adjustments |
| **Battery**   | Power source for ESP32, motors, and sensors |
| **Chassis & Wheels** | Physical structure of the bot |

---

## ⚙️ Circuit Connections

### **MPU6050 to ESP32:**
- **VCC** → 3.3V (ESP32)
- **GND** → GND (ESP32)
- **SDA** → GPIO21 (ESP32)
- **SCL** → GPIO22 (ESP32)

### **Motor Driver to ESP32:**
- **IN1, IN2** → GPIO26, GPIO27 (ESP32)
- **IN3, IN4** → GPIO14, GPIO12 (ESP32)
- **Enable Pins (PWM)** → GPIO33, GPIO32 (ESP32)

---

## 🔢 PID Control Implementation

The **PID (Proportional-Integral-Derivative) controller** ensures smooth and stable balancing by adjusting motor speed based on tilt error.

```cpp
float Kp = 15.0, Ki = 0.2, Kd = 1.5;
float error, previousError, integral, derivative;

void loop() {
    int16_t ax, ay, az, gx, gy, gz;
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    float angle = atan2(ax, az) * 180 / PI;
    
    error = angle;
    integral += error;
    derivative = error - previousError;
    int motorSpeed = Kp * error + Ki * integral + Kd * derivative;
    
    controlMotors(motorSpeed);
    previousError = error;
    delay(10);
}
```

---

## 🛠 Setup & Installation

1. Clone the repository:
   ```sh
   git clone https://github.com/yourusername/SelfBalancing-Bot-ESP32.git
   ```
2. Open the code in **Arduino IDE** or **PlatformIO**.
3. Install the required libraries:
   - `Wire.h`
   - `MPU6050.h`
4. Upload the code to ESP32 and power up the bot!

---

## 🚀 Future Enhancements
- Implementing a **Kalman Filter** for improved sensor fusion.
- Adding **Bluetooth control** for remote tuning.
- Integrating an **OLED display** for real-time sensor data.

---

## 🤝 Contributions
Feel free to fork, enhance, and create pull requests! Contributions are always welcome.

---

## 📝 License
This project is **open-source** under the MIT License.

---

### 📧 Contact
For any queries, reach out to me on **[LinkedIn](https://www.linkedin.com/in/achyuth-mukund)** or via email at **achyuth2004@gmail.com**.
