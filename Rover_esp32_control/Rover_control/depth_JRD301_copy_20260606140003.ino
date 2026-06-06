#include <Arduino.h>

// --- MOTOR CONFIGURATION ---
const int pwmPins[4] = {33, 22, 27, 5};
const int dirPins[4] = {18, 14, 25, 23};
const int forwardState[4] = {HIGH, LOW, HIGH, LOW};

// --- STATE VARIABLES ---
String currentCommand = "BOOTING";
int currentPWM = 0;
unsigned long lastCommandTime = 0; 
unsigned long lastTelemetryTime = 0;

void setup() {
  Serial.begin(115200);

  // Setup Motors 
  for (int i = 0; i < 4; i++) {
    ledcAttach(pwmPins[i], 5000, 8);   
    pinMode(dirPins[i], OUTPUT);
    digitalWrite(dirPins[i], LOW);
  }
}

void moveForward() {
  
  
  digitalWrite(dirPins[1], forwardState[1]);
  digitalWrite(dirPins[2], forwardState[2]);
  digitalWrite(dirPins[3], forwardState[3]);    
  ledcWrite(pwmPins[1], 30);
  ledcWrite(pwmPins[2], 40); 
  ledcWrite(pwmPins[3], 40); 
  
}
void moveleft() {
  
  
  digitalWrite(dirPins[1], forwardState[1]);
  digitalWrite(dirPins[2], forwardState[2]);
  digitalWrite(dirPins[3], forwardState[3]);    
  ledcWrite(pwmPins[1], 22);
  ledcWrite(pwmPins[2], 65); 
  ledcWrite(pwmPins[3], 22); 
  
}
void moveright() {
  
  
  digitalWrite(dirPins[1], forwardState[1]);
  digitalWrite(dirPins[2], forwardState[2]);
  digitalWrite(dirPins[3], forwardState[3]);    
  ledcWrite(pwmPins[1], 90); 
  ledcWrite(pwmPins[2], 8); 
  ledcWrite(pwmPins[3], 36); 
  
}

void stopMotors() {
  currentPWM = 0;
  for (int i = 0; i < 4; i++) {
    ledcWrite(pwmPins[i], currentPWM); 
  }
}

void loop() {
  // 1. LISTEN FOR JETSON COMMANDS
  if (Serial.available()) {
    currentCommand = Serial.readStringUntil('\n');
    currentCommand.trim();
    lastCommandTime = millis(); 

    if (currentCommand == "MOVE") {
      moveForward();
    }
    else if (currentCommand == "LEFT") {
      moveleft();
    }
    else if (currentCommand == "RIGHT") {
      moveright();
    }
    else if (currentCommand == "STOP") {
      stopMotors();
    }
  }

  // 2. WATCHDOG FAILSAFE
  if (millis() - lastCommandTime > 500) {
    stopMotors();
    if (currentCommand != "STOP") {
        currentCommand = "WATCHDOG_STOP"; // Alerts you if connection is lost
    }
  }

  // 3. SEND TELEMETRY TO JETSON (10Hz)
  // Sends data incredibly fast over the wire without blocking the motor loop
  if (millis() - lastTelemetryTime > 100) {
    Serial.print("TEL:");
    Serial.print(currentCommand);
    Serial.print(",");
    Serial.println(currentPWM);
    
    lastTelemetryTime = millis();
  }
}