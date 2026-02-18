#include <WiFiNINA.h>
#include <Arduino_LSM6DS3.h>

WiFiClient client;

const int buttonPin = 21;

char ssid[] = "DigitalTeknik24";
char pass[] = "DigiTek24";

char bridgeIP[] = "192.168.50.206";
char hueUser[] = "iCozq5nVqXP9mV0b19NTbGdNR05d4VkQQhkW2SCK";
int lightID = 38;

float roll = 0.0;
float pitch = 0.0;
float yaw = 0.0;

unsigned long lastTime;
unsigned long lastHueSend = 0;

int currentBri = 10;
int currentHue = 0;

const float briChangePerSec = (254.0 - 10.0) / 2.0;
const float hueChangePerSec = 5000.0;

bool systemOn = true;
bool lastButtonState = HIGH;


// ===== Send til Hue =====
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
  unsigned long start = millis();
  while (!Serial && millis() - start < 5000);

  Serial.println("BOOT OK");

  Serial.println("Connecting WiFi...");
  WiFi.begin(ssid, pass);

  unsigned long t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 10000) {
    Serial.print(".");
    delay(500);
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWiFi OK");
  } else {
    Serial.println("\nWiFi FAIL (lokal drift stadig ok)");
  }

  if (!IMU.begin()) {
    Serial.println("IMU fejl!");
    while (1);
  }

  lastTime = millis();
  Serial.println("SYSTEM READY");
  Serial.println("Pitch\tYaw\tBri\tHue");
}

void loop() {

  // ===== KNAP =====
  bool buttonState = digitalRead(buttonPin);

  if (lastButtonState == HIGH && buttonState == LOW) {
    systemOn = !systemOn;

    if (systemOn) {
      Serial.println("SYSTEM ON");
      lastTime = millis();
    } else {
      Serial.println("SYSTEM OFF");
      setLamp(false, 0, 0);
    }

    delay(300);
  }

  lastButtonState = buttonState;

  if (!systemOn) return;

  float ax, ay, az;
  float gx, gy, gz;

  if (IMU.accelerationAvailable() && IMU.gyroscopeAvailable()) {

    IMU.readAcceleration(ax, ay, az);
    IMU.readGyroscope(gx, gy, gz);

    unsigned long now = millis();
    float dt = (now - lastTime) / 1000.0;
    lastTime = now;

    // === GYRO integration ===
    roll  += gx * dt;
    pitch += gy * dt;
    yaw   += gz * dt + 0.76 * dt;

    // === ACC correction ===
    float rollAcc  = atan2(ay, az) * 180.0 / PI;
    float pitchAcc = atan2(-ax, sqrt(ay * ay + az * az)) * 180.0 / PI;

    roll  = 0.98 * roll  + 0.02 * rollAcc;
    pitch = 0.98 * pitch + 0.02 * pitchAcc;

    // ===== BRIGHTNESS fra pitch =====
    if (pitch > 30) currentBri += briChangePerSec * dt;
    else if (pitch < -30) currentBri -= briChangePerSec * dt;
    currentBri = constrain(currentBri, 10, 254);

    // ===== HUE fra yaw =====
    if (yaw > 30) currentHue += hueChangePerSec * dt;
    else if (yaw < -30) currentHue -= hueChangePerSec * dt;
    currentHue = constrain(currentHue, 0, 65535);

    // ===== SEND =====
    if (millis() - lastHueSend > 200) {
      lastHueSend = millis();
      setLamp(true, currentHue, currentBri);
    }

    // ===== SERIAL DEBUG (NU MED YAW) =====
    Serial.print(pitch); Serial.print('\t');
    Serial.print(yaw);   Serial.print('\t');
    Serial.print(currentBri); Serial.print('\t');
    Serial.println(currentHue);
  }
}
