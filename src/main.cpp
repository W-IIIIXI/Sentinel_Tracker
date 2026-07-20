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
float lastWindG = 0.0; // cached so /telemetry doesn't have to touch the IMU directly

// --- GPS & SUN-PATH VARIABLES ---
// These used to be hardcoded consts. They are now plain globals ("dynamic")
// because the dashboard's /calibrate POST overwrites them at runtime. The
// values below only act as a fallback until the first calibration arrives.
float LATITUDE  = 6.9271;
float LONGITUDE = 79.8612;
float TIMEZONE_OFFSET = 5.5;
bool  isCalibrated = false; // flips true once the dashboard has calibrated us at least once

float targetPanAngle  = 0.0;
float targetTiltAngle = 0.0;

// --- MANUAL OVERRIDE STATE (driven by the D-pad in the dashboard) ---
// The dashboard just sends discrete pan_left/pan_right/tilt_up/tilt_down
// pulses, so we track the commanded angles here and let Module 3's motor
// driver chase these values, the same way it will chase targetPanAngle/
// targetTiltAngle in auto mode.
float manualPanAngle  = 0.0;
float manualTiltAngle = 0.0;
const float MANUAL_STEP_DEG = 1.0; // degrees moved per D-pad pulse

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

const char* stateToString(TrackerState s) {
  switch (s) {
    case STATE_HIBERNATION:       return "HIBERNATION";
    case STATE_TRACKING:          return "TRACKING";
    case STATE_WIND_STOW:         return "WIND_STOW";
    case STATE_OVERCURRENT_FAULT: return "OVERCURRENT_FAULT";
    case STATE_MANUAL_OVERRIDE:   return "MANUAL_OVERRIDE";
    default:                      return "UNKNOWN";
  }
}

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

// TODO(hardware): wire these up to real ADC channels once the panel-voltage
// divider and current-sense resistor are on the board. Returning safe
// placeholder values for now so /telemetry has something to serve and the
// dashboard doesn't sit on "OFFLINE".
float getPanelVoltage() {
  return 0.0;
}

float getMotorCurrentMa() {
  return 0.0;
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

// --- WEB SERVER ROUTES ---

void handleTelemetry(AsyncWebServerRequest *request) {
  StaticJsonDocument<128> doc;
  doc["voltage"] = getPanelVoltage();
  doc["wind"]    = lastWindG;
  doc["current"] = getMotorCurrentMa();

  String payload;
  serializeJson(doc, payload);
  request->send(200, "application/json", payload);
}

void handleCommand(AsyncWebServerRequest *request) {
  if (!request->hasParam("action")) {
    request->send(400, "text/plain", "missing action");
    return;
  }
  String action = request->getParam("action")->value();

  if (action == "set_mode_manual") {
    currentState = STATE_MANUAL_OVERRIDE;
    // Seed manual angles from wherever auto-tracking last pointed, so
    // switching modes doesn't cause a sudden jump.
    manualPanAngle  = targetPanAngle;
    manualTiltAngle = targetTiltAngle;

  } else if (action == "set_mode_auto") {
    currentState = isCalibrated ? STATE_TRACKING : STATE_HIBERNATION;

  } else if (action == "pan_left") {
    if (currentState == STATE_MANUAL_OVERRIDE) manualPanAngle -= MANUAL_STEP_DEG;

  } else if (action == "pan_right") {
    if (currentState == STATE_MANUAL_OVERRIDE) manualPanAngle += MANUAL_STEP_DEG;

  } else if (action == "tilt_up") {
    if (currentState == STATE_MANUAL_OVERRIDE) manualTiltAngle += MANUAL_STEP_DEG;

  } else if (action == "tilt_down") {
    if (currentState == STATE_MANUAL_OVERRIDE) manualTiltAngle -= MANUAL_STEP_DEG;

  } else if (action == "force_stow") {
    currentState = STATE_WIND_STOW; // existing stow logic drives to 0 degrees

  } else if (action == "e_stop") {
    currentState = STATE_OVERCURRENT_FAULT; // reuse the fault state as a hard lock

  } else {
    request->send(400, "text/plain", "unknown action");
    return;
  }

  request->send(200, "text/plain", "ok");
}

// Body handler for POST /calibrate. The dashboard sends:
// { "lat": 6.9271, "lon": 79.8612, "utcOffset": 5.5,
//   "year": 2026, "month": 7, "day": 20,
//   "hour": 14, "minute": 30, "second": 5 }
void handleCalibrate(AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total) {
  StaticJsonDocument<256> doc;
  DeserializationError err = deserializeJson(doc, data, len);

  if (err) {
    request->send(400, "text/plain", "bad json");
    return;
  }

  if (!doc.containsKey("lat") || !doc.containsKey("lon")) {
    request->send(400, "text/plain", "missing lat/lon");
    return;
  }

  // --- Dynamic variables: overwrite the sun-path constants in RAM ---
  LATITUDE        = doc["lat"].as<float>();
  LONGITUDE       = doc["lon"].as<float>();
  TIMEZONE_OFFSET = doc["utcOffset"] | TIMEZONE_OFFSET; // keep old value if field absent

  // --- Static variable: push the wall-clock time into the DS3231 hardware RTC ---
  if (doc.containsKey("year") && doc.containsKey("month") && doc.containsKey("day") &&
      doc.containsKey("hour") && doc.containsKey("minute") && doc.containsKey("second")) {
    DateTime newTime(
      doc["year"].as<int>(),
      doc["month"].as<int>(),
      doc["day"].as<int>(),
      doc["hour"].as<int>(),
      doc["minute"].as<int>(),
      doc["second"].as<int>()
    );
    rtc.adjust(newTime);
  }

  isCalibrated = true;

  // Once we know where and when we are, it's safe to start tracking
  // (unless something else, like manual override or wind stow, is active).
  if (currentState == STATE_HIBERNATION) {
    currentState = STATE_TRACKING;
  }

  Serial.printf("Calibrated: lat=%.4f lon=%.4f tz=%.2f\n", LATITUDE, LONGITUDE, TIMEZONE_OFFSET);

  request->send(200, "application/json", "{\"status\":\"ok\"}");
}

void setupWebServer() {
  // Serve the dashboard itself
  server.serveStatic("/", LittleFS, "/").setDefaultFile("index.html");

  server.on("/telemetry", HTTP_GET, handleTelemetry);
  server.on("/command", HTTP_GET, handleCommand);

  // POST /calibrate has a body, so it needs the 3-arg body handler,
  // registered as the body callback of a no-op onRequest handler.
  server.on("/calibrate", HTTP_POST,
    [](AsyncWebServerRequest *request) { /* handled in body callback below */ },
    NULL,
    handleCalibrate
  );

  server.begin();
  Serial.println("Web server started.");
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

  // 6. Bring up the dashboard + API routes (was missing: server.begin() too)
  setupWebServer();
}

void loop() {
  // 1. NON-BLOCKING SENSOR POLLING (The Sentinel is always watching)
  if (millis() - lastSensorPoll >= SENSOR_POLL_RATE_MS) {
    lastSensorPoll = millis();

    lastWindG = getWindVibration();

    // Safety Override Check (The Virtual Anemometer)
    if (lastWindG > 1.5 && currentState != STATE_WIND_STOW) {
      Serial.println("CRITICAL WIND DETECTED! Overriding FSM to STOW MODE.");
      currentState = STATE_WIND_STOW;
    }
  }
  
  // 2. FINITE STATE MACHINE EXECUTION
  switch (currentState) {
    
    case STATE_HIBERNATION:
      // Code to ensure motors are de-energized
      // Waits for calibration data from the dashboard (or ADC-based
      // daylight detection later) before moving to STATE_TRACKING.
      break;

    case STATE_TRACKING:
      // Constantly crunch the math to update targetPanAngle and targetTiltAngle
      // using the (possibly dashboard-calibrated) LATITUDE/LONGITUDE/TIMEZONE_OFFSET
      calculateSunPath();
      
      // (Module 3: Code to drive motors to match these angles will go here)
      break;

    case STATE_WIND_STOW:
      // Override logic to flatten the panel to 0 degrees
      targetPanAngle = 0.0;
      targetTiltAngle = 0.0;
      break;

    case STATE_OVERCURRENT_FAULT:
      // Complete system lock until reset
      break;

    case STATE_MANUAL_OVERRIDE:
      // Dashboard D-pad control logic: manualPanAngle / manualTiltAngle are
      // updated in handleCommand(); Module 3's motor driver should chase
      // these instead of targetPanAngle/targetTiltAngle while in this state.
      break;
  }

  delay(10); // Small delay for ESP32 stability
}