#include <WiFiNINA.h>
#include <Arduino_LSM6DS3.h>

WiFiClient client;

// ===== PIN =====
const int buttonPin = 26;

// ===== WIFI =====
char ssid[] = "DigitalTeknik24";
char pass[] = "DigiTek24";

// ===== HUE =====
char bridgeIP[] = "192.168.50.206";
char hueUser[] = "41oxODjOX9EsuUQo5KgwbglIOZQxXIItIOAmJdV7";
int lightID = 23;

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

// ===== SYSTEM STATE =====
bool systemOn = true;
bool lastButtonState = HIGH;

// ===== Hue send =====
void setLamp(bool on, int hue, int bri) {
  if (client.connect(bridgeIP, 80)) {

    String body;

    if (on) {
      body = "{\"on\":true,\"hue\":" + String(hue) +
             ",\"sat\":254,\"bri\":" + String(bri) + "}";
    } else {
      body = "{\"on\":false}";
    }

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

  lastTime = millis();
  Serial.println("System ON");
}

void loop() {

  // ===== BUTTON TOGGLE =====
  bool buttonState = digitalRead(buttonPin);

  if (lastButtonState == HIGH && buttonState == LOW) {
    systemOn = !systemOn;

    if (systemOn) {
      Serial.println("System ON");
      lastTime = millis();
    } else {
      Serial.println("System OFF");
      setLamp(false, 0, 0);  // sluk Hue-lampen
    }

    delay(300); // debounce
  }

  lastButtonState = buttonState;

  // ===== STOP ALT HVIS SLUKKET =====
  if (!systemOn) {
    return;
  }

  // ===== NORMAL IMU + HUE =====
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

    // Brightness (Y)
    if (pitch > 30) currentBri += briChangePerSec * dt;
    else if (pitch < -30) currentBri -= briChangePerSec * dt;
    currentBri = constrain(currentBri, 10, 254);

    // Hue (Z)
    if (yaw > 30) currentHue += hueChangePerSec * dt;
    else if (yaw < -30) currentHue -= hueChangePerSec * dt;
    currentHue = constrain(currentHue, 0, 65535);

    // Send til Hue
    if (millis() - lastHueSend > 200) {
      lastHueSend = millis();
      setLamp(true, currentHue, currentBri);
    }

    // Debug
    Serial.print("Pitch: "); Serial.print(pitch);
    Serial.print("  Bri: "); Serial.print(currentBri);
    Serial.print("  Hue: "); Serial.println(currentHue);
  }
}
