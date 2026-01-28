#include <WiFiNINA.h>
#include <Arduino_LSM6DS3.h>

WiFiClient client;

// ===== WIFI =====
char ssid[] = "DigitalTeknik24";
char pass[] = "DigiTek24";

// ===== HUE =====
char bridgeIP[] = "192.168.50.206";
char hueUser[] = "iCozq5nVqXP9mV0b19NTbGdNR05d4VkQQhkW2SCK"; // Din API-nøgle
int lightID = 38;   // A11 23 Color

// ===== IMU =====
float roll = 0.0;
float pitch = 0.0;
float yaw = 0.0;

unsigned long lastTime;
unsigned long lastHueSend = 0;

// Current lamp values
int currentBri = 10;      // lysstyrke (10-254)
int currentHue = 0;       // farve (0-65535)

// Settings for smooth changes
const float briChangePerSec = (254.0 - 10.0) / 2.0;  // når det tager 5 sekunder
const float hueChangePerSec = 15000.0;                // farve ændring per sekund

void setLamp(int hue, int bri) {
  if (client.connect(bridgeIP, 80)) {
    String body = "{\"on\":true,\"hue\":" + String(hue) + ",\"sat\":254,\"bri\":" + String(bri) + "}";
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
  Serial.begin(9600);
  unsigned long start = millis();
  while (!Serial && (millis() - start < 5000));

  // WiFi
  Serial.println("Connecting to WiFi...");
  WiFi.begin(ssid, pass);
  unsigned long t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 10000) {
    delay(500);
    Serial.print(".");
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWiFi connected!");
    Serial.print("IP: "); Serial.println(WiFi.localIP());
  } else {
    Serial.println("\nWiFi connection failed");
  }

  // IMU
  Serial.println("Initializing IMU...");
  if (!IMU.begin()) {
    Serial.println("IMU init failed!");
  } else {
    Serial.println("IMU ready!");
  }

  lastTime = millis();
  Serial.println("Roll\tPitch\tYaw\tBri\tHue");
}

void loop() {
  float ax, ay, az;
  float gx, gy, gz;

  if (IMU.accelerationAvailable() && IMU.gyroscopeAvailable()) {
    IMU.readAcceleration(ax, ay, az);
    IMU.readGyroscope(gx, gy, gz);

    unsigned long now = millis();
    float dt = (now - lastTime) / 1000.0; // delta-tid i sekunder
    lastTime = now;

    // === GYRO INTEGRATION ===
    roll  += gx * dt;
    pitch += gy * dt;
    yaw   += gz * dt;

    // === ACCELEROMETER ABSOLUT VINKLER ===
    float rollAcc  = atan2(ay, az) * 180.0 / PI;
    float pitchAcc = atan2(-ax, sqrt(ay * ay + az * az)) * 180.0 / PI;

    // === COMPLEMENTARY FILTER ===
    roll  = 0.98 * roll  + 0.02 * rollAcc;
    pitch = 0.98 * pitch + 0.02 * pitchAcc;

    // === Smooth brightness (pitch/Y) ===
    if (pitch > 40) {
      currentBri += briChangePerSec * dt;
    } else if (pitch < -40) {
      currentBri -= briChangePerSec * dt;
    }
    currentBri = constrain(currentBri, 10, 254);

    // === Smooth hue (yaw/Z) ===
    if (yaw > 40) {
      currentHue += hueChangePerSec * dt;
    } else if (yaw < -40) {
      currentHue -= hueChangePerSec * dt;
    }
    currentHue = constrain(currentHue, 0, 65535);

    // Send to lamp max 5 gange per sekund
    if (millis() - lastHueSend > 200) {
      lastHueSend = millis();
      setLamp(currentHue, currentBri);
    }

    // Print til Serial Monitor
    Serial.print(roll);
    Serial.print('\t');
    Serial.print(pitch);
    Serial.print('\t');
    Serial.print(yaw);
    Serial.print('\t');
    Serial.print(currentBri);
    Serial.print('\t');
    Serial.println(currentHue);

    delay(50);
  }
}
