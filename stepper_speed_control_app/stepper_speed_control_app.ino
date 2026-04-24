#include "BluetoothSerial.h"
BluetoothSerial SerialBT;

#define STEP_PIN 27
#define DIR_PIN 26

// ── Continuous-mode state ────────────────────────────────────────────────────
long current_freq = 0;
bool runMotor = true;

// ── Step-angle-mode state ────────────────────────────────────────────────────
bool stepMode = false;
long totalPulses = 0;
long currentPulse = 0;
long stepFreq = 0;           // Target frequency (Hz)
int lastProgressPercent = -1;

// ── Acceleration ramp state ─────────────────────────────────────────────────
// Motor can't start at full speed from standstill — it needs a gradual ramp.
#define STEP_START_FREQ  200L     // Hz — slow enough for any stepper to start
#define ACCEL_RATE       8000L    // Hz increase per second (linear ramp)

long  currentToneFreq = 0;        // Current tone frequency being output
double estimatedPulses = 0.0;     // Running integral of pulses generated
unsigned long lastStepUpdateMicros = 0;

// ── Direction tracking ───────────────────────────────────────────────────────
bool currentDirForward = true;

String cmd = "";

void setup() {
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);

  digitalWrite(DIR_PIN, HIGH);

  Serial.begin(115200);
  SerialBT.begin("Stepper_Control");

  Serial.println("System Ready");
}

// -------- CONTINUOUS MOTOR UPDATE --------
void updateMotor() {
  if (!stepMode && runMotor && current_freq > 0) {
    tone(STEP_PIN, current_freq);
  } else if (!stepMode) {
    noTone(STEP_PIN);
  }
}

// -------- STEP-ANGLE WITH ACCELERATION RAMP --------
// tone() generates the pulses in hardware.
// This function:
//   1. Ramps the tone frequency from STEP_START_FREQ up to stepFreq
//   2. Tracks pulses by integrating the instantaneous frequency over time
//   3. Reports progress and stops when totalPulses have been generated
void updateStepMode() {
  if (!stepMode || totalPulses <= 0 || stepFreq <= 0) return;

  unsigned long now = micros();
  unsigned long dt = now - lastStepUpdateMicros;
  lastStepUpdateMicros = now;

  // Guard against huge dt (first call, micros overflow, or BT stall)
  if (dt > 50000) dt = 50000;  // Cap at 50ms

  // ── Accumulate pulses at the CURRENT tone frequency ─────────────────────
  estimatedPulses += (double)currentToneFreq * dt / 1000000.0;
  currentPulse = (long)estimatedPulses;

  // ── Accelerate: linearly ramp frequency toward target ───────────────────
  if (currentToneFreq < stepFreq) {
    long freqIncrease = (long)((double)dt * ACCEL_RATE / 1000000.0);
    if (freqIncrease < 1) freqIncrease = 1;
    long newFreq = currentToneFreq + freqIncrease;
    if (newFreq > stepFreq) newFreq = stepFreq;
    if (newFreq != currentToneFreq) {
      currentToneFreq = newFreq;
      tone(STEP_PIN, currentToneFreq);
    }
  }

  // ── Progress reporting every 5% ─────────────────────────────────────────
  int percent = (int)((currentPulse * 100L) / totalPulses);
  if (percent > 100) percent = 100;
  if (percent != lastProgressPercent && (percent % 5 == 0)) {
    lastProgressPercent = percent;
    SerialBT.println("STEP_PROGRESS:" + String(currentPulse) + "/" + String(totalPulses));
    Serial.println("Progress: " + String(percent) + "% @ " + String(currentToneFreq) + " Hz");
  }

  // ── Check if done ───────────────────────────────────────────────────────
  if (currentPulse >= totalPulses) {
    stepMode = false;
    noTone(STEP_PIN);             // Stop hardware PWM
    digitalWrite(STEP_PIN, LOW);  // Ensure pin is LOW
    runMotor = false;             // Prevent continuous mode restart
    currentPulse = totalPulses;
    SerialBT.println("STEP_DONE");
    Serial.println("Step sequence complete (" + String(totalPulses) + " pulses)");
  }
}

// -------- STATUS REPORTER --------
void sendStatus() {
  String status = "STATUS:run=";
  status += (runMotor ? "1" : "0");
  status += ",freq=" + String(current_freq);
  status += ",dir=" + String(currentDirForward ? "F" : "R");
  status += ",step=" + String(stepMode ? "1" : "0");
  status += ",spulse=" + String(currentPulse);
  status += ",tpulse=" + String(totalPulses);
  SerialBT.println(status);
  Serial.println("Sent: " + status);
}

void loop() {

  // -------- BLUETOOTH --------
  while (SerialBT.available()) {
    char c = SerialBT.read();

    if (c == '\n') {
      cmd.trim();
      processCommand(cmd);
      cmd = "";
    } else {
      cmd += c;
    }
  }

  // Step-angle progress tracking + acceleration (non-blocking)
  updateStepMode();
}

// -------- COMMAND HANDLER --------
void processCommand(String cmd) {

  // FIX: Remove the literal '\n' characters that the Flutter app is sending
  cmd.replace("\\n", "");

  Serial.println("Received: " + cmd);

  // -------- DIRECTION --------
  if (cmd == "F") {
    bool wasRunning = runMotor;      
    runMotor = false;                
    updateMotor();                   // STOP pulsing
    if (wasRunning) delay(300);      
    digitalWrite(DIR_PIN, HIGH);
    currentDirForward = true;
    delay(10);                       
    runMotor = wasRunning;           
    updateMotor();
  }

  else if (cmd == "R") {
    bool wasRunning = runMotor;
    runMotor = false;
    updateMotor();
    if (wasRunning) delay(300);
    digitalWrite(DIR_PIN, LOW);
    currentDirForward = false;
    delay(10);
    runMotor = wasRunning;
    updateMotor();
  }

  // -------- START / STOP --------
  else if (cmd == "G") {
    runMotor = true;
    updateMotor();
  }

  else if (cmd == "S") {
    runMotor = false;
    updateMotor();
  }

  // -------- SPEED (Frequency) --------
  else if (cmd.startsWith("FRQ:")) {
    long freqHz = cmd.substring(4).toInt();
    if (freqHz < 0) freqHz = 0;
    current_freq = freqHz;
    updateMotor();
  }

  // -------- STEP ANGLE COMMAND: STEP:angle,rotations,ppr,speed_hz --------
  else if (cmd.startsWith("STEP:")) {
    String params = cmd.substring(5);
    int c1 = params.indexOf(',');
    int c2 = params.indexOf(',', c1 + 1);
    int c3 = params.indexOf(',', c2 + 1);

    if (c1 > 0 && c2 > 0 && c3 > 0) {
      float angle = params.substring(0, c1).toFloat();
      int rotations = params.substring(c1 + 1, c2).toInt();
      int ppr = params.substring(c2 + 1, c3).toInt();
      long freq = params.substring(c3 + 1).toInt();

      // Calculate total pulses needed
      totalPulses = (long)((angle / 360.0) * ppr * rotations);
      currentPulse = 0;
      stepFreq = freq;
      lastProgressPercent = -1;

      if (totalPulses > 0 && stepFreq > 0) {
        // Stop continuous mode first
        noTone(STEP_PIN);
        runMotor = false;

        // Start step mode with acceleration ramp
        stepMode = true;
        estimatedPulses = 0.0;
        currentToneFreq = STEP_START_FREQ;
        lastStepUpdateMicros = micros();
        tone(STEP_PIN, currentToneFreq);  // Start slow — ramp up in updateStepMode()

        SerialBT.println("STEP_START:" + String(totalPulses));
        Serial.println("Step mode: " + String(totalPulses) + " pulses, target " + String(stepFreq) + " Hz, accel from " + String(STEP_START_FREQ) + " Hz");
      }
    }
  }

  // -------- EMERGENCY STOP (step mode) --------
  else if (cmd == "STEP_STOP") {
    stepMode = false;
    noTone(STEP_PIN);
    digitalWrite(STEP_PIN, LOW);
    SerialBT.println("STEP_STOPPED:" + String(currentPulse) + "/" + String(totalPulses));
    Serial.println("Step mode emergency stopped");
  }

  // -------- STATUS QUERY --------
  else if (cmd == "STATUS") {
    sendStatus();
  }
}
