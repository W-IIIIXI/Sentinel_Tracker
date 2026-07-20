#include <Arduino.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <LittleFS.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include <RTClib.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

// --- Soft-AP Credentials (The Off-Grid Network) ---
const char* ssid = "Sentinel_Tracker_01";
const char* password = "adminpassword";
AsyncWebServer server(80);

// --- I2C SENSOR OBJECTS ---
RTC_DS3231 rtc;
Adafruit_MPU6050 mpu;

// --- VIRTUAL ANEMOMETER VARIABLES ---
float previousAccelMag = 0.0;
unsigned long lastSensorPoll = 0;
const int SENSOR_POLL_RATE_MS = 50; // Poll MPU-6050 at 20Hz

// --- GPS & SUN-PATH VARIABLES ---
const float LATITUDE = 6.9271; 
const float LONGITUDE = 79.8612;
const float TIMEZONE_OFFSET = 5.5; 

float targetPanAngle = 0.0;
float targetTiltAngle = 0.0;

// --- FINITE STATE MACHINE DEFINITION ---
enum TrackerState {
  STATE_HIBERNATION,       // Panel shaded, motors off
  STATE_TRACKING,          // Sun is out, calculating angles
  STATE_WIND_STOW,         // High wind detected, driving to 0 degrees
  STATE_OVERCURRENT_FAULT, // Motor jam detected, system locked
  STATE_MANUAL_OVERRIDE    // User controlling via Soft-AP Wi-Fi
};

// Initialize in Hibernation mode to save power on boot
TrackerState currentState = STATE_HIBERNATION;

// --- SENSOR READING FUNCTIONS ---
DateTime getCurrentTime() {
  // Queries the DS3231 for exact UTC/Local time for astronomical math
  return rtc.now();
}

float getWindVibration() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // Calculate the 3D magnitude of the current acceleration vector
  float currentAccelMag = sqrt(pow(a.acceleration.x, 2) + 
                               pow(a.acceleration.y, 2) + 
                               pow(a.acceleration.z, 2));

  // The "High-Pass Filter": Isolate the sudden change (shudder)
  float deltaG = abs(currentAccelMag - previousAccelMag);
  
  // Save current magnitude for the next loop comparison
  previousAccelMag = currentAccelMag;

  // Convert raw acceleration (m/s^2) to G-force for easier thresholding
  return deltaG / 9.81; 
}

// --- ASTRONOMICAL MATH FUNCTION ---
void calculateSunPath() {
  DateTime now = getCurrentTime();
  
  // Calculate day of the year (1-365) and decimal hour
  int dayOfYear = now.unixtime() / 86400 % 365;
  float hour = now.hour() + (now.minute() / 60.0);

  // 1. Solar Declination
  float declination = 23.45 * sin(radians((360.0 / 365.0) * (dayOfYear - 81.0)));

  // 2. Equation of Time (EoT) approximation in minutes
  float B = radians((360.0 / 365.0) * (dayOfYear - 81.0));
  float eot = 9.87 * sin(2 * B) - 7.53 * cos(B) - 1.5 * sin(B);

  // 3. True Solar Time (TST)
  float timeOffset = (LONGITUDE * 4.0) - (TIMEZONE_OFFSET * 60.0);
  float tst = hour + (timeOffset + eot) / 60.0;

  // 4. Hour Angle (HRA)
  float hra = 15.0 * (tst - 12.0);

  // 5. Elevation Angle (Tilt)
  float elevation = degrees(asin(sin(radians(declination)) * sin(radians(LATITUDE)) + 
                           cos(radians(declination)) * cos(radians(LATITUDE)) * cos(radians(hra))));

  // 6. Azimuth Angle (Pan)
  float azimuth = degrees(acos((sin(radians(declination)) * cos(radians(LATITUDE)) - 
                          cos(radians(declination)) * sin(radians(LATITUDE)) * cos(radians(hra))) / 
                          cos(radians(elevation))));

  // Correct Azimuth based on time of day (post-solar noon)
  if (hra > 0) {
    azimuth = 360.0 - azimuth;
  }

  // Update global targets
  targetTiltAngle = max(0.0f, elevation); // Prevent driving panel into the ground
  targetPanAngle = azimuth;
}

void setup() {
  Serial.begin(115200);
  Serial.println("\n--- Continuous Sentinel Boot ---");

  // 1. Initialize LittleFS to serve your dashboard
  if(!LittleFS.begin(true)){
    Serial.println("An Error has occurred while mounting LittleFS");
  }

  // 2. Initialize I2C Bus (Standard ESP32 pins 21 and 22)
  Wire.begin(21, 22); 

  // 3. Boot the DS3231 RTC
  if (!rtc.begin()) {
    Serial.println("CRITICAL: Couldn't find RTC!");
  } else {
    Serial.println("DS3231 RTC Initialized.");
    if (rtc.lostPower()) {
      Serial.println("RTC lost power, setting time to compile time!");
      rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    }
  }

  // 4. Boot the MPU-6050 IMU
  if (!mpu.begin()) {
    Serial.println("CRITICAL: Failed to find MPU6050 chip!");
  } else {
    Serial.println("MPU-6050 IMU Initialized.");
    mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  }

  // 5. Start the Soft-AP Wi-Fi Network
  WiFi.softAP(ssid, password);
  Serial.print("AP IP address: ");
  Serial.println(WiFi.softAPIP()); 
}

void loop() {
  // 1. NON-BLOCKING SENSOR POLLING (The Sentinel is always watching)
  if (millis() - lastSensorPoll >= SENSOR_POLL_RATE_MS) {
    lastSensorPoll = millis();

    float currentWindG = getWindVibration();

    // Safety Override Check (The Virtual Anemometer)
    if (currentWindG > 1.5 && currentState != STATE_WIND_STOW) {
      Serial.println("CRITICAL WIND DETECTED! Overriding FSM to STOW MODE.");
      currentState = STATE_WIND_STOW;
    }
  }
  
  // 2. FINITE STATE MACHINE EXECUTION
  switch (currentState) {
    
    case STATE_HIBERNATION:
      // Code to ensure motors are de-energized
      // (Transition to STATE_TRACKING will happen based on solar panel ADC voltage later)
      break;

    case STATE_TRACKING:
      // Constantly crunch the math to update targetPanAngle and targetTiltAngle
      calculateSunPath();
      
      // (Module 3: Code to drive motors to match these angles will go here)
      break;

    case STATE_WIND_STOW:
      // Override logic to flatten the panel to 0 degrees
      break;

    case STATE_OVERCURRENT_FAULT:
      // Complete system lock until reset
      break;

    case STATE_MANUAL_OVERRIDE:
      // Dashboard joystick control logic
      break;
  }

  delay(10); // Small delay for ESP32 stability
}
