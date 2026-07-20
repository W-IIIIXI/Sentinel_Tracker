# ☀️ SENTINEL: Off-Grid Predictive Dual-Axis Solar Tracker

[![PlatformIO](https://img.shields.io/badge/PlatformIO-ESP32-orange.svg)](https://platformio.org/)
[![License: MIT](https://img.shields.io/badge/License-MIT-blue.svg)](LICENSE)

**Sentinel** is an ultra-efficient, intelligent dual-axis solar tracking system engineered specifically for high-altitude, moderately forested environments (such as the Sri Lankan hill country). 

While the physical structure utilizes a standard, cost-effective generic dual-axis assembly, the true engineering novelty lies within its **Intelligent Energy-Positive Firmware Architecture**.

---

## 🧠 System Architecture & Software Novelty

Conventional trackers rely on reactive Light Dependent Resistors (LDRs) that constantly "hunt" for scattered light through dense tree canopies, wasting more battery power than the panel generates. **Sentinel** replaces reactive sensing with an advanced firmware-driven approach:

* **Predictive Astronomical Tracking:** Utilizes a DS3231 Real-Time Clock (RTC) combined with onboard sun-path mathematical modeling (Equation of Time and Hour Angle calculations) to precisely align with the sun, ignoring dappled canopy shadows.
* **Energy-Aware Finite State Machine (FSM):** Continuously samples solar panel voltage. If shading conditions mean actuation energy would exceed harvest yield, the system mathematically aborts movement and enters a low-power hibernation state.
* **Virtual Anemometer (Wind Defense):** Employs an MPU-6050 IMU polled at 20Hz. A custom high-pass filter isolates high-frequency structural flutter, triggering a hardware override into a flat 0° aerodynamic "Stow Mode" during severe windstorms.
* **Dead-Reckoning & Homing:** Eliminates magnetic compass interference (from nearby motors) by utilizing physical limit switches to establish a precise mechanical reference at True East and Level, tracking subsequent movements through precise kinematic timing.

---

## 📱 Localized Soft-AP Mobile Dashboard

To ensure total operational independence in off-grid terrain where internet access is unavailable, Sentinel avoids cloud-based IoT services entirely:

* **Direct Wi-Fi Broadcast:** The ESP32 acts as a standalone Software Access Point (`Sentinel_Tracker_01`).
* **Asynchronous Web Server:** Built using `ESPAsyncWebServer`, serving a responsive single-page web application from internal flash memory (`LittleFS`) without interrupting core safety control loops.
* **Field Calibration API:** Features a custom `/calibrate` POST route allowing field technicians to sync mobile GPS coordinates and local atomic clock time directly into the tracker's hardware without needing a laptop.
* **Manual Override & E-STOP:** Provides an industrial hold-to-steer D-pad and a hardware-level safety de-energization sequence.

---

## 🗂️ Repository Structure

This project is built and managed using **PlatformIO** in Visual Studio Code:

```text
Sentinel_Tracker/
│
├── platformio.ini         # Core environment and automated library dependencies
├── src/
│   └── main.cpp           # C++ Backend (FSM, I2C sensor fusion, Async Web Server)
└── data/
    └── index.html         # Vanilla HTML/CSS/JS Mobile Admin Dashboard

(as of 20.07.2026)
