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

// -----------------------------
// Manufacturer Data Packet (19 bytes)
// -----------------------------
//
// The manufacturer-specific data packet is always 19 bytes long.
//  0-1: Company ID (0x02, 0x01)
//  2:   Version Major
//  3:   Version Minor
//  4:   Data Type
//  5:   Equipment ID
//  6-7: Cadence (unsigned 16-bit little-endian)
//  8-9: Heart Rate (unsigned 16-bit little-endian)
// 10-11: Power (unsigned 16-bit little-endian)
// 12-13: Caloric Burn (unsigned 16-bit little-endian)
// 14:   Duration Minutes (1 byte)
// 15:   Duration Seconds (1 byte)
// 16-17: Distance (unsigned 16-bit little-endian); if is_metric is true, the MSB is set.
// 18:   Gear (1 byte)
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
  Serial.print("  Version: "); Serial.print(version_major); Serial.print("."); Serial.println(version_minor);
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
void simulateKeiserM3BLE(uint8_t equipment_id, unsigned long durationSec) {
  unsigned long startTime = millis();
  unsigned long elapsed = 0;
  uint16_t cadence = 0, heart_rate = 0, power = 0, total_calories = 0, total_distance = 0;
  uint8_t gear = 1;

  while (elapsed < durationSec * 1000UL) {
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

    delay(1000);
    elapsed = millis() - startTime;
  }
}

// -----------------------------
// Arduino setup() and loop()
// -----------------------------
void setup() {
  Serial.begin(115200);
  while (!Serial) { ; }
  randomSeed(analogRead(A0));

  BLEDevice::init("M3");
  pAdvertising = BLEDevice::getAdvertising();
  advPayload.setName("M3");
  pAdvertising->setScanResponse(false);
  pAdvertising->setAdvertisementData(advPayload);
  pAdvertising->start();

  Serial.println("Starting Keiser M3 simulation with BLE...");
  simulateKeiserM3BLE(56, 30);
  Serial.println("Simulation complete.");
}

void loop() {
  // Nothing to do in loop.
}
