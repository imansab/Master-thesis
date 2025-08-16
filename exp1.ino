/*
  Author: Sabbani Assabban Iman
  Date: 15/07/2025
  Project: Contribution of Demand Response to Frequency Control: 
  Frequency Measurement and Control using Analog Input
  Description:
    - This program reads an analog signal from pin A1.
    - It detects rising edges to compute the frequency in real time.
    - A moving average filter smooths the signal.
    - A digital output is controlled based on the filtered frequency using hysteresis.
    - Serial output displays timestamp, frequency, and state.
*/

const int analogPin = A1;        // Analog input pin (A1)
const int threshold = 200;       // Detection threshold (~0.6V for a 3.3V supply)

const int outputPin = 7;         // Digital output pin (e.g., LED, relay, etc.)

const int N = 10;                // Filter size for moving average
float freqBuffer[N] = {0};       // Circular buffer for last N frequency measurements
int bufIndex = 0;                // Current index in the buffer
bool bufferFull = false;         // True when the buffer has been filled at least once

bool previousState = false;      // Previous state of the analog signal (above/below threshold)
unsigned long lastTime = 0;      // Time of the last rising edge (in microseconds)
unsigned long period = 0;        // Measured period between two rising edges (in microseconds)

void setup() {
  Serial.begin(115200);                 // Initialize serial communication
  while (!Serial);                     // Wait for serial connection (if required)

  pinMode(outputPin, OUTPUT);          // Set output pin as OUTPUT
  digitalWrite(outputPin, LOW);        // Start with output OFF

  Serial.println("Time (ms),Frequency (Hz),State"); // CSV header for serial plot
}

void loop() {
  int val = analogRead(analogPin);              // Read analog signal
  bool aboveThreshold = val > threshold;        // Determine if signal is above threshold

  // Detect rising edge (from below threshold to above)
  if (!previousState && aboveThreshold) {
    unsigned long now = micros();               // Get current time in microseconds
    period = now - lastTime;                    // Time elapsed since last rising edge
    lastTime = now;                             // Update last time

    if (period > 0) {
      float freq = 1e6 / period;                // Compute frequency in Hz

      // Store frequency in circular buffer
      freqBuffer[bufIndex++] = freq;
      if (bufIndex == N) {
        bufIndex = 0;
        bufferFull = true;
      }

      if (bufferFull) {
        // Compute filtered frequency (moving average)
        float sum = 0;
        for (int i = 0; i < N; i++) {
          sum += freqBuffer[i];
        }
        float filteredFreq = sum / N;

        // Hysteresis control of output state
        static bool currentState = false; // false = OFF, true = ON

        if (filteredFreq >= 49.9 && !currentState) {
          currentState = true;
          digitalWrite(outputPin, HIGH);  // Turn output ON
        } else if (filteredFreq <= 49.7 && currentState) {
          currentState = false;
          digitalWrite(outputPin, LOW);   // Turn output OFF
        }

        // Serial output for logging
        float time_ms = millis();         // Current time in milliseconds
        Serial.print(time_ms, 2);
        Serial.print(",");
        Serial.print(filteredFreq, 2);
        Serial.print(",");
        Serial.println(currentState ? "ON" : "OFF");
      }
    }
  }

  previousState = aboveThreshold;       // Update previous signal state
  delayMicroseconds(100);               // Sampling delay (~10 kHz rate)
}