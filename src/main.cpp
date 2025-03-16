/*
   This Arduino sketch simulates the Keiser M3 data transmission,
   generating manufacturer-specific data and BLE advertising data packets.
   Now extended to broadcast over BLE using the ESP32 BLE library.
*/

#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEAdvertising.h>
#include <string.h>
#include <stdlib.h>
#include <HTTPClient.h>
#ifdef BUILD_FULL
#include <WebSerial.h>
#endif
#include <WiFi.h>
#include <Preferences.h>
Preferences preferences;
#ifdef BUILD_FULL
#include <ESPAsyncWebServer.h>
#endif

BLEAdvertising* pAdvertising;
BLEAdvertisementData advPayload;

// -----------------------------
// Data conversion and helper functions
// -----------------------------
//
// Helper: Write a little-endian 16-bit value into a byte array
void write_uint16_le(uint16_t value, uint8_t *dest) {
  dest[0] = value & 0xFF;
  dest[1] = (value >> 8) & 0xFF;
}

void generateM3Data(uint8_t equipment_id, uint8_t version_major, uint8_t version_minor, uint8_t data_type,
                    uint16_t cadence, uint16_t heart_rate, uint16_t power, uint16_t caloric_burn,
                    uint8_t duration_minutes, uint8_t duration_seconds,
                    uint16_t distance, uint8_t gear, bool is_metric, uint8_t data[19]) {
  data[0] = 0x02; data[1] = 0x01;
  data[2] = version_major; data[3] = version_minor; data[4] = data_type; data[5] = equipment_id;
  write_uint16_le(cadence, &data[6]);
  write_uint16_le(heart_rate, &data[8]);
  write_uint16_le(power, &data[10]);
  write_uint16_le(caloric_burn, &data[12]);
  data[14] = duration_minutes;
  data[15] = duration_seconds;
  uint16_t distance_val = distance;
  if (is_metric) distance_val |= 0x8000;
  write_uint16_le(distance_val, &data[16]);
  data[18] = gear;
}

// -----------------------------
// Parse Manufacturer Data
// -----------------------------
//
// This function reads the 19-byte manufacturer data and prints the parsed values.
void parseManufacturerData(const uint8_t data[19]) {
  if (data[0] != 0x02 || data[1] != 0x01) {
    Serial.println("Invalid Company ID");
    #ifdef BUILD_FULL
    WebSerial.println("Invalid Company ID");
    #endif
    return;
  }

  uint8_t version_major = data[2];
  uint8_t version_minor = data[3];
  uint8_t data_type = data[4];
  uint8_t equipment_id = data[5];
  uint16_t cadence = data[6] | (((uint16_t)data[7]) << 8);
  uint16_t heart_rate = data[8] | (((uint16_t)data[9]) << 8);
  uint16_t power = data[10] | (((uint16_t)data[11]) << 8);
  uint16_t caloric_burn = data[12] | (((uint16_t)data[13]) << 8);
  uint8_t duration_minutes = data[14];
  uint8_t duration_seconds = data[15];
  uint16_t dist_raw = data[16] | (((uint16_t)data[17]) << 8);
  bool is_metric = (dist_raw & 0x8000) != 0;
  uint16_t distance = dist_raw & 0x7FFF;
  uint8_t gear = data[18];

  Serial.println("Parsed Manufacturer Data:");
  Serial.print("  Version: "); 
  #ifdef BUILD_FULL
    WebSerial.print("  Version: ");
  #endif
  Serial.print(version_major); Serial.print("."); Serial.println(version_minor);
  Serial.print("  Data Type: "); Serial.println(data_type);
  Serial.print("  Equipment ID: "); Serial.println(equipment_id);
  Serial.print("  Cadence: "); Serial.println(cadence / 10.0, 1);
  Serial.print("  Heart Rate: "); Serial.println(heart_rate / 10.0, 1);
  Serial.print("  Power: "); Serial.println(power);
  Serial.print("  Caloric Burn: "); Serial.println(caloric_burn);
  Serial.print("  Duration: "); Serial.print(duration_minutes); Serial.print(" minutes, "); Serial.print(duration_seconds); Serial.println(" seconds");
  Serial.print("  Distance: "); Serial.println(distance / 10.0, 1);
  Serial.print("  Units: "); Serial.println(is_metric ? "Metric" : "Imperial");
  Serial.print("  Gear: "); Serial.println(gear);
}

// -----------------------------
// BLE Advertisement Update
// -----------------------------
//
// Replaces previous advertisement with new manufacturer data
void updateBLEAdvertisement(const uint8_t manufacturer_data[19]) {
  std::string m_data((char*)manufacturer_data, 19);
  advPayload.setManufacturerData(m_data);
  pAdvertising->stop();
  pAdvertising->setAdvertisementData(advPayload);
  pAdvertising->start();
}

// -----------------------------
// Simulation of Keiser M3 Data Transmission with BLE
// Now also prints local Bluetooth MAC address
// -----------------------------
void simulateKeiserM3BLE(uint8_t equipment_id) {
  Preferences preferences;
  while (true) {
    unsigned long startTime = millis();
    unsigned long elapsed = 0;
    uint16_t cadence = 0, heart_rate = 0, power = 0, total_calories = 0, total_distance = 0;
    uint8_t gear = 1;

    while (elapsed < 30000) {
      cadence = (uint16_t)(random(60, 101) * 10);
      heart_rate = (uint16_t)(random(120, 151) * 10);
      power = (uint16_t)random(100, 201);
      gear = (uint8_t)random(1, 25);
      total_calories += (power * 2) / 10;
      total_distance += (cadence * 5) / 100;
      bool is_metric = true;

      uint8_t m_data[19];
      uint8_t dur_min = (uint8_t)((elapsed / 60000UL) % 256);
      uint8_t dur_sec = (uint8_t)((elapsed / 1000UL) % 60);

      generateM3Data(equipment_id, 6, 30, 0, cadence, heart_rate, power, total_calories,
                    dur_min, dur_sec, total_distance, gear, is_metric, m_data);

      updateBLEAdvertisement(m_data);

      Serial.print("MAC Address: ");
      Serial.println(BLEDevice::getAddress().toString().c_str());
      Serial.print("IP Address: ");
      Serial.println(WiFi.localIP());
      Serial.print("Time: ");
      Serial.print(elapsed / 1000.0, 1);
      Serial.println(" s");
      Serial.print("  Advertising Data: ");
      for (int i = 0; i < 19; i++) {
        if (m_data[i] < 16) Serial.print("0");
        Serial.print(m_data[i], HEX);
      }
      Serial.println();
      parseManufacturerData(m_data);
      Serial.println("-------------------------");

      if (WiFi.status() == WL_CONNECTED) {
        HTTPClient http;
        http.begin("https://influx.glencook.tech/api/v2/write?org=glencook&bucket=keiser_data&precision=s");
        preferences.begin("influx", false);
        String influxToken = preferences.getString("token", "");
        preferences.end();
        http.addHeader("Authorization", "Token " + influxToken);
        http.addHeader("Content-Type", "text/plain");

        String line = String("keiser_metrics,device=M3 cadence=") + String(cadence/10.0, 1) +
                      ",heartrate=" + String(heart_rate/10.0, 1) +
                      ",power=" + String(power) +
                      ",calories=" + String(total_calories) +
                      ",duration_min=" + String(dur_min) +
                      ",duration_sec=" + String(dur_sec) +
                      ",distance=" + String(total_distance / 10.0, 1) +
                      ",gear=" + String(gear);

        int httpCode = http.POST(line);
        if (httpCode > 0) {
          Serial.println("[InfluxDB] Sent: " + line);
          Serial.println("[InfluxDB] Response: " + String(httpCode));
        } else {
          Serial.println("[InfluxDB] Failed to send: " + http.errorToString(httpCode));
        }
        http.end();
      }

      delay(1000);
      elapsed = millis() - startTime;
    }
  }
}

// -----------------------------
// Arduino setup() and loop()
// -----------------------------
// Wi-Fi Credentials (will load from preferences or use defaults)


String ssid = "";
String password = "";

#ifdef BUILD_FULL
AsyncWebServer server(80);
#endif

void setup() {
  // Connect to Wi-Fi
  preferences.begin("wifi", false);
  ssid = preferences.getString("ssid", "BeastCave");
  password = preferences.getString("password", "w0mb@t9078");
  preferences.end();

  Serial.print("Connecting to SSID: ");
  Serial.println(ssid);

  WiFi.begin(ssid.c_str(), password.c_str());
  int attempts = 0;
  WiFi.setTxPower(WIFI_POWER_2dBm);
  while (WiFi.status() != WL_CONNECTED && attempts < 20) {
    delay(500);
    Serial.print(".");
    attempts++;
  }

  if (WiFi.status() != WL_CONNECTED) {
    WiFi.mode(WIFI_MODE_AP);
    WiFi.softAP("KeiserSim_AP");
    IPAddress IP = WiFi.softAPIP();
    Serial.println("Wi-Fi connection failed. Starting fallback AP...");
    Serial.print("AP IP address: ");
    Serial.println(IP);
    #ifdef BUILD_FULL
    WebSerial.println("Wi-Fi connection failed. Starting fallback AP...");
#endif
    #ifdef BUILD_FULL
    WebSerial.print("Access Point started. Connect to SSID 'KeiserSim_AP'. IP: ");
#endif
    Serial.print("Access Point started. Connect to SSID 'KeiserSim_AP'. IP: ");
    Serial.println(IP);
  } else {
    Serial.println();
    Serial.print("Connected! IP address: ");
    Serial.println(WiFi.localIP());
  }
  Serial.println();
  Serial.print("Connected! IP address: ");
  Serial.println(WiFi.localIP());

  // Start WebSerial
  #ifdef BUILD_FULL
  WebSerial.begin(&server);
  WebSerial.onMessage([](uint8_t *data, size_t len) {
    String input = "";
    for (size_t i = 0; i < len; i++) input += (char)data[i];
    WebSerial.println("Received: " + input);

    if (input.startsWith("token:")) {
      String token = input.substring(6);
      preferences.begin("influx", false);
      preferences.putString("token", token);
      preferences.end();
      WebSerial.println("Saved InfluxDB token.");
    }
  });
#endif
  #ifdef BUILD_FULL
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    String html = "<!DOCTYPE html><html><head><meta name='viewport' content='width=device-width, initial-scale=1'>"
                  "<style>body{font-family:sans-serif;padding:2em;}input{width:100%;padding:0.5em;margin:0.5em 0;}button{padding:0.7em;width:100%;background:#007bff;color:#fff;border:none;cursor:pointer;}</style>"
                  "</head><body><h2>Wi-Fi Config</h2>"
                  "<form action='/save' method='get'>"
                  "Token: <input name='token' placeholder='InfluxDB Token'><br>" 
"SSID: <input name='ssid' placeholder='Wi-Fi Name'><br>"
                  "Password: <input name='pass' type='password' placeholder='Wi-Fi Password'><br>"
                  "<button type='submit'>Save and Reboot</button><br><p style='color:red;'>Simulation will only start if Wi-Fi is connected and a token is provided.</p></form></body></html>";
    request->send(200, "text/html", html);
  });
#endif

  #ifdef BUILD_FULL
  server.on("/save", HTTP_GET, [](AsyncWebServerRequest *request){
    if (request->hasParam("ssid") && request->hasParam("pass")) {
      String token = request->hasParam("token") ? request->getParam("token")->value() : "";
      ssid = request->getParam("ssid")->value();
      password = request->getParam("pass")->value();
      preferences.begin("wifi", false);
      preferences.putString("ssid", ssid);
      preferences.putString("password", password);
      if (token != "") preferences.putString("token", token);
      preferences.end();
      request->send(200, "text/plain", "Saved. Rebooting...");
      delay(1000);
      ESP.restart();
    } else {
      request->send(400, "text/plain", "Missing parameters");
    }
  });
#endif

  #ifdef BUILD_FULL
  server.begin();
#endif
  // Captive portal redirect
  #ifdef BUILD_FULL
  server.onNotFound([](AsyncWebServerRequest *request){
    request->redirect("/");
  });
#endif
  #ifdef BUILD_FULL
  WebSerial.println("WebSerial started. Open serial monitor in browser.");
#endif
  Serial.begin(115200);
  while (!Serial) { ; }
  randomSeed(analogRead(A0));

  BLEDevice::init("M3");
  pAdvertising = BLEDevice::getAdvertising();
  advPayload.setName("M3");
  pAdvertising->setScanResponse(false);
  pAdvertising->setAdvertisementData(advPayload);
  pAdvertising->start();

  // Wait until Wi-Fi is fully ready or fallback AP is confirmed
  int waitWiFi = 0;
  while (WiFi.status() != WL_CONNECTED && waitWiFi < 10000) {
    delay(100);
    waitWiFi += 100;
  }

  preferences.begin("influx", false);
  String tokenCheck = preferences.getString("token", "");
  preferences.end();

  if (WiFi.status() == WL_CONNECTED && tokenCheck.length() > 0) {
    // Simulation only starts when Wi-Fi and token are available
    Serial.println("Starting Keiser M3 simulation with BLE...");
    simulateKeiserM3BLE(56);
    Serial.println("Simulation complete.");
  } else {
    Serial.println("Wi-Fi not connected or missing InfluxDB token. BLE simulation not started.");
  }
}

void loop() {
  // Nothing to do in loop.
}
