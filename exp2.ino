/*
  Author: Sabbani Assabban Iman
  Date: 15/07/2025
  Project: Contribution of Demand Response to Frequency Control  
  Title: Frequency Measurement and Temperature-Based Control

  Description:
    - This program reads an analog signal from pin A1.
    - It detects rising edges to compute the frequency in real time.
    - A moving average filter smooths the frequency signal.
    - A digital output controls a heater based on both filtered frequency 
      (for demand response) and internal temperature (for safety).
    - Safety lockout periods avoid rapid switching.
    - Serial output logs timestamp, frequency, temperature, state, and decision reason.
*/

// --- Pin configuration and initial heater state ---
const int analogPin = A1;              // Analog input pin for frequency signal
const int seuil = 200;                 // Threshold to detect rising edges
const int sortiePin = 7;               // Digital output pin to control the heating element
bool heatingON = false;                // Current state of the heating system

// --- Frequency measurement using a circular buffer ---
const int N = 10;                      // Number of samples for moving average
float freqBuffer[N] = {0};            // Buffer to store the last N frequency values
int bufIndex = 0;                      // Current index in the buffer
bool bufferFull = false;              // Indicates whether the buffer is fully filled
bool previousState = false;           // Used to detect rising edges
unsigned long lastTime = 0;           // Time of the previous rising edge
unsigned long period = 0;             // Measured period between rising edges

// --- Thermal model parameters ---
float T = 55;                          // Initial internal temperature of the boiler (°C)
float T_env = 20.0;                    // Ambient room temperature (°C)
float R = 2.5;                         // Thermal resistance (°C/W)
float C = 41860.0;                     // Thermal capacity (J/°C)
float P_in = 1836.0;                   // Power input when heater is ON (W)
float dt = 1.0;                        // Time step for integration (s)
float T_max = 70.0;                    // Maximum allowed temperature (°C)
float T_min = 65.0;                    // Minimum allowed temperature (°C)
unsigned long lastUpdate = 0;         // Last time temperature was updated

// --- Frequency control parameters ---
float f_min = 49.98;                   // Lower frequency threshold (Hz)
float f_max = 49.99;                   // Upper frequency threshold (Hz)
float error = 0.01;                    // Tolerance for frequency deadband (Hz)
bool freqState = true;                 // Control state based on frequency (ON/OFF)

// --- Lockout timers to prevent rapid switching ---
unsigned long lastOFF = 0;            // Time when heater was last turned OFF
unsigned long lastON  = 0;            // Time when heater was last turned ON
const unsigned long lockOUT_OFF = 500; // Minimum time between OFF and next ON (ms)
const unsigned long lockOUT_ON  = 500; // Minimum time between ON and next OFF (ms)

// --- Decision control timing ---
unsigned long lastDecision = 0;       // Last time a frequency-based decision was taken
const unsigned long decisionWindow = 1000; // Time interval to re-evaluate frequency (ms)

void setup() {
  Serial.begin(115200);               // Start serial communication
  while (!Serial);                    // Wait for serial connection (for some boards)
  pinMode(sortiePin, OUTPUT);         // Set output pin as output
  digitalWrite(sortiePin, LOW);       // Start with heating OFF
  Serial.println("Time (ms),Freq (Hz),Temp (°C),State,Reason"); // CSV log header
}

void loop() {
  int val = analogRead(analogPin);              // Read analog signal
  bool aboveThreshold = val > seuil;            // Rising edge detection condition

  if (!previousState && aboveThreshold) {       // If rising edge is detected
    unsigned long now = micros();               // Current time in microseconds
    period = now - lastTime;                    // Calculate period between pulses
    lastTime = now;

    if (period > 0) {
      float freq = 1e6 / period;                // Convert period to frequency (Hz)
      freqBuffer[bufIndex++] = freq;            // Store in buffer
      if (bufIndex == N) {
        bufIndex = 0;
        bufferFull = true;
      }

      if (bufferFull) {
        float sum = 0;
        for (int i = 0; i < N; i++) sum += freqBuffer[i];
        float filteredFreq = sum / N;           // Calculate moving average

        // --- Update temperature every 1 second ---
        if (millis() - lastUpdate >= 1000) {
          lastUpdate = millis();
          float P = heatingON ? P_in : 0.0;     // Power input depends on heating state
          float dTdt = (P - (T - T_env) / R) / C;
          T += dt * dTdt;                       // Integrate using Euler method
        }

        String reason = "OK";                   // Reason string for logging

        // --- Safety control: override frequency logic if temperature is critical ---
        if (T < T_min) {
          heatingON = true;
          lastON = millis();
          reason = "T < Tmin → ON";
        } else if (T >= T_max) {
          if ((millis() - lastON) >= lockOUT_ON) {
            heatingON = false;
            lastOFF = millis();
            reason = "T ≥ Tmax → OFF";
          } else {
            reason = "T ≥ Tmax but ON lockout active";
          }
        }
        // --- Frequency control logic evaluated every decisionWindow ---
        else if (millis() - lastDecision >= decisionWindow) {
          lastDecision = millis();

          if (filteredFreq <= f_min - error) {
            freqState = false;
            lastOFF = millis();
            reason = "Freq ≤ lower threshold → OFF";
          }
          else if (filteredFreq >= f_max + error) {
            if ((millis() - lastOFF) >= lockOUT_OFF) {
              freqState = true;
              reason = "Freq ≥ upper threshold + lockout → ON";
            } else {
              reason = "Freq ≥ upper threshold but OFF lockout active";
            }
          }
          else {
            reason = freqState ? "Deadband (holding ON)" : "Deadband (holding OFF)";
          }

          // --- Apply decision from frequency logic if temperature allows ---
          if (freqState) {
            if (!heatingON) {
              lastON = millis();
              heatingON = true;
              reason = "Freq OK → ON";
            }
          } else {
            if (heatingON && (millis() - lastON) >= lockOUT_ON) {
              heatingON = false;
              lastOFF = millis();
              reason = "Freq decision → OFF";
            }
          }
        }

        // --- Update output pin and log to serial monitor ---
        digitalWrite(sortiePin, heatingON ? HIGH : LOW);   // Apply heating state
        Serial.print(millis()); Serial.print(",");
        Serial.print(filteredFreq, 4); Serial.print(",");
        Serial.print(T, 2); Serial.print(",");
        Serial.print(heatingON ? "ON" : "OFF"); Serial.print(",");
        Serial.println(reason);                             // Print log
      }
    }
  }

  previousState = aboveThreshold;      // Update previous signal state for next loop
  delayMicroseconds(100);              // Small delay to reduce CPU usage
}