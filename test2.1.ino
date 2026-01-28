#include <WiFiNINA.h>
#include <Arduino_LSM6DS3.h>
#include <ArduinoLowPower.h>

WiFiClient client;

// ===== PIN =====
const int buttonPin = 2;

// ===== WIFI =====
char ssid[] = "DigitalTeknik24";
char pass[] = "DigiTek24";

// ===== HUE =====
char bridgeIP[] = "192.168.50.206";
char hueUser[] = "41oxODjOX9EsuUQo5KgwbglIOZQxXIItIOAmJdV7";
int lightID = 38;

// ===== IMU =====
float roll = 0.0;
float pitch = 0.0;
float yaw = 0.0;

unsigned long lastTime;
unsigned long lastHueSend = 0;

// Lamp state
int currentBri = 10;
int currentHue = 0;

const float briChangePerSec = (254.0 - 10.0) / 5.0;
const float hueChangePerSec = 5000.0;

// ===== Sleep flag =====
bool sleeping = false;

// ===== Hue send =====
void setLamp(int hue, int bri) {
  if (client.connect(bridgeIP, 80)) {
    String body = "{\"on\":true,\"hue\":" + String(hue) +
                  ",\"sat\":254,\"bri\":" + String(bri) + "}";
    client.println("PUT /api/" + String(hueUser) +
                   "/lights/" + String(lightID) + "/state HTTP/1.1");
    client.println("Host: " + String(bridgeIP));
    client.println("Content-Type: application/json");
    client.print("Content-Length: ");
    client.println(body.length());
    client.println();
    client.println(body);
    client.stop();
  }
}

// ===== Wakeup ISR =====
void wakeUp() {
  // tom – bare nødvendigt for interrupt
}

void setup() {
  pinMode(buttonPin, INPUT_PULLUP);

  Serial.begin(9600);
  delay(2000);

  Serial.println("Booting...");

  // WiFi
  WiFi.begin(ssid, pass);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected");

  // IMU
  if (!IMU.begin()) {
    Serial.println("IMU fejl!");
    while (1);
  }

  // Enable wakeup on button
  LowPower.attachInterruptWakeup(
    buttonPin,
    wakeUp,
    FALLING
  );

  lastTime = millis();
  Serial.println("Roll\tPitch\tYaw\tBri\tHue");
}

void loop() {

  // === Button → Sleep ===
  if (digitalRead(buttonPin) == LOW) {
    Serial.println("Going to sleep...");
    delay(500); // debounce
    WiFi.end();
    LowPower.deepSleep();
    // ↓ Koden fortsætter her efter wakeup ↓
    Serial.println("Woke up!");
    lastTime = millis();
  }

  float ax, ay, az;
  float gx, gy, gz;

  if (IMU.accelerationAvailable() && IMU.gyroscopeAvailable()) {
    IMU.readAcceleration(ax, ay, az);
    IMU.readGyroscope(gx, gy, gz);

    unsigned long now = millis();
    float dt = (now - lastTime) / 1000.0;
    lastTime = now;

    roll  += gx * dt;
    pitch += gy * dt;
    yaw   += gz * dt;

    float rollAcc  = atan2(ay, az) * 180.0 / PI;
    float pitchAcc = atan2(-ax, sqrt(ay * ay + az * az)) * 180.0 / PI;

    roll  = 0.98 * roll  + 0.02 * rollAcc;
    pitch = 0.98 * pitch + 0.02 * pitchAcc;

    // Brightness
    if (pitch > 30) currentBri += briChangePerSec * dt;
    else if (pitch < -30) currentBri -= briChangePerSec * dt;
    currentBri = constrain(currentBri, 10, 254);

    // Hue
    if (yaw > 30) currentHue += hueChangePerSec * dt;
    else if (yaw < -30) currentHue -= hueChangePerSec * dt;
    currentHue = constrain(currentHue, 0, 65535);

    if (millis() - lastHueSend > 200) {
      lastHueSend = millis();
      setLamp(currentHue, currentBri);
    }

    Serial.print(roll); Serial.print('\t');
    Serial.print(pitch); Serial.print('\t');
    Serial.print(yaw); Serial.print('\t');
    Serial.print(currentBri); Serial.print('\t');
    Serial.println(currentHue);
  }
}
