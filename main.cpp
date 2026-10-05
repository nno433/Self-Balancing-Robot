#include <Arduino.h>
#include <Wire.h>
#include "MPU6050.h"
#include <WiFi.h>
#include <WebServer.h>

// --- PIN CONFIGURATION ---
#define PWMA 13   
#define AIN1 14   
#define AIN2 12   
#define STBY_PIN 27 
#define BIN1 26   
#define BIN2 25   
#define PWMB 33   
#define BATTERY_PIN 34
#define SDA_PIN 21
#define SCL_PIN 22

// --- WIFI CONFIGURATION ---
const char* ssid = "---------------------------";
const char* password = "--------------------------";

WebServer server(80);
MPU6050 mpu;

// --- PID TUNING PARAMETERS ---
volatile double Kp = 100.0;  
volatile double Ki = 0.0; 
volatile double Kd = 0.0;   

volatile double targetAngle = 1.0; 
double integrityError = 0.0;
double lastError = 0.0;

// --- PROPORTIONAL MOVEMENT OVERLAYS ---
volatile double moveOffsetAngle = 0.0; 
volatile int turnOffset = 0;           
volatile unsigned long lastCommandTime = 0; // Fixes the "ghost joystick" runaway issue

// --- FILTER & TELEMETRY ---
float alpha = 0.98; 
float robotAngle = 0;

volatile float globalBatteryVoltage = 7.4;
volatile float globalCurrentAngle = 0.0;
volatile int globalWiFiRSSI = 0; 

// --- RTOS MUTEX FOR THREAD SAFETY ---
SemaphoreHandle_t xDataMutex;

TaskHandle_t BalanceTaskHandle = NULL;
TaskHandle_t TelemetryTaskHandle = NULL;

void vBalanceTask(void *pvParameters);
void vTelemetryTask(void *pvParameters);
void initMotors();
void moveMotors(int leftSpeed, int rightSpeed);
void handleTelemetry();
void handleSetPID();
void handleControl();
void sendCORSHeaders();

void setup() {
    Serial.begin(115200);
    
    Wire.begin(SDA_PIN, SCL_PIN, 400000);
    Wire.setTimeOut(20); 

    xDataMutex = xSemaphoreCreateMutex();

    Serial.println("Initializing MPU6050...");
    mpu.initialize();
    if (!mpu.testConnection()) {
        Serial.println("MPU6050 connection failed!");
        while (1);
    }

    // Apply calibrated sensor offsets
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788);

    initMotors();

    WiFi.begin(ssid, password);
    Serial.print("Connecting to WiFi");
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("\nConnected! IP Address: ");
    Serial.println(WiFi.localIP());

    server.on("/telemetry", HTTP_GET, handleTelemetry);
    server.on("/set_pid", HTTP_GET, handleSetPID); 
    server.on("/control", HTTP_GET, handleControl); 
    
    server.onNotFound([]() {
        sendCORSHeaders();
        if (server.method() == HTTP_OPTIONS) {
            server.sendHeader("Access-Control-Allow-Methods", "POST, GET, OPTIONS");
            server.sendHeader("Access-Control-Allow-Headers", "Content-Type");
            server.send(204);
        } else {
            server.send(404, "text/plain", "Not found");
        }
    });
    server.begin();

    xTaskCreatePinnedToCore(vBalanceTask, "BalanceTask", 4096, NULL, 3, &BalanceTaskHandle, 1);
    xTaskCreatePinnedToCore(vTelemetryTask, "TelemetryTask", 4096, NULL, 1, &TelemetryTaskHandle, 0);
}

void loop() {
    vTaskDelete(NULL); 
}

void sendCORSHeaders() {
    server.sendHeader("Access-Control-Allow-Origin", "*");
    server.sendHeader("Access-Control-Allow-Methods", "GET, POST, OPTIONS");
}

// --- API ENDPOINTS ---
void handleTelemetry() {
    sendCORSHeaders();
    String json = "{";
    if (xSemaphoreTake(xDataMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        json += "\"angle\":" + String(globalCurrentAngle) + ",";
        json += "\"battery\":" + String(globalBatteryVoltage) + ",";
        json += "\"rssi\":" + String(globalWiFiRSSI) + ","; 
        json += "\"Kp\":" + String(Kp) + ",";
        json += "\"Ki\":" + String(Ki) + ",";
        json += "\"Kd\":" + String(Kd) + ",";
        json += "\"targetAngle\":" + String(targetAngle);
        xSemaphoreGive(xDataMutex);
    }
    json += "}";
    server.send(200, "application/json", json);
}

void handleSetPID() {
    sendCORSHeaders();
    if (xSemaphoreTake(xDataMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        if (server.hasArg("Kp")) Kp = server.arg("Kp").toDouble();
        if (server.hasArg("Ki")) Ki = server.arg("Ki").toDouble();
        if (server.hasArg("Kd")) Kd = server.arg("Kd").toDouble();
        if (server.hasArg("Target")) targetAngle = server.arg("Target").toDouble();
        integrityError = 0; 
        xSemaphoreGive(xDataMutex);
    }
    server.send(200, "text/plain", "OK");
}

void handleControl() {
    sendCORSHeaders();
    if (xSemaphoreTake(xDataMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        if (server.hasArg("move")) moveOffsetAngle = server.arg("move").toDouble();
        if (server.hasArg("turn")) turnOffset = server.arg("turn").toInt();
        lastCommandTime = millis(); // Refresh safety countdown clock on every message
        xSemaphoreGive(xDataMutex);
    }
    server.send(200, "text/plain", "OK");
}

// --- CORE CALCULATIONS LOOP ---
void vBalanceTask(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const float dt = 0.005; 
    const TickType_t xFrequency = pdMS_TO_TICKS(5); 

    double localKp, localKi, localKd, localTarget, localMove;
    int localTurn;

    while (1) {
        int16_t ax, ay, az, gx, gy, gz;
        mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

        // --- FIXED: AUTOMATIC SENSOR FREEZE DETECTOR & BUS RECOVERY ---
        static int16_t last_ax, last_ay, last_az;
        static int freezeCounter = 0;

        if (ax == last_ax && ay == last_ay && az == last_az) {
            freezeCounter++;
        } else {
            freezeCounter = 0;
        }
        last_ax = ax; last_ay = ay; last_az = az;

        if (freezeCounter > 50) { // If raw values perfectly duplicate for 250ms
            Wire.end(); 
            vTaskDelay(pdMS_TO_TICKS(5));
            Wire.begin(SDA_PIN, SCL_PIN, 400000); 
            Wire.setTimeOut(20);
            mpu.initialize(); 
            // Reapply critical offsets so it doesn't immediately lurch sideways
            mpu.setXGyroOffset(220);
            mpu.setYGyroOffset(76);
            mpu.setZGyroOffset(-85);
            mpu.setZAccelOffset(1788);
            freezeCounter = 0;
            continue; 
        }

        // Thread-Safe Data Fetching
        if (xSemaphoreTake(xDataMutex, 0) == pdTRUE) {
            localKp = Kp; localKi = Ki; localKd = Kd;
            localTarget = targetAngle; 
            
            // --- FIXED: DEAD-MAN'S SAFETY SWITCH ---
            // If driving via app and connection is dropped for > 1 second, clear commands
            if (millis() - lastCommandTime > 1000 && (moveOffsetAngle != 0 || turnOffset != 0)) {
                moveOffsetAngle = 0; 
                turnOffset = 0;      
            }
            
            localMove = moveOffsetAngle;
            localTurn = turnOffset;
            xSemaphoreGive(xDataMutex);
        }

        float accAngle = atan2(ay, az) * 180.0 / PI; 
        float gyroRate = gx / 131.0; 

        robotAngle = alpha * (robotAngle + gyroRate * dt) + (1.0 - alpha) * accAngle;
        globalCurrentAngle = robotAngle;

        float error = (localTarget + localMove) - robotAngle;
        
        if (abs(robotAngle) < 25) { 
            integrityError += error * dt;
            integrityError = constrain(integrityError, -100, 100); 
        } else {
            integrityError = 0; 
        }
        
        float derivative = (error - lastError) / dt;
        lastError = error;

        float output = (localKp * error) + (localKi * integrityError) + (localKd * derivative);

        // Progressive Gearbox Friction Deadzone
        const int deadzone = 45; 
        if (abs(output) > 0.1) {
            output += (output > 0) ? deadzone : -deadzone;
        }

        if (abs(robotAngle) > 40.0 || globalBatteryVoltage < 6.2) {
            moveMotors(0, 0);
            digitalWrite(STBY_PIN, LOW); 
            integrityError = 0;
        } else {
            digitalWrite(STBY_PIN, HIGH);
            int baseSpeed = constrain((int)output, -255, 255);
            moveMotors(baseSpeed + localTurn, baseSpeed - localTurn);
        }

        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

void vTelemetryTask(void *pvParameters) {
    pinMode(BATTERY_PIN, INPUT);
    analogSetPinAttenuation(BATTERY_PIN, ADC_11db);

    while (1) {
        server.handleClient();
        int rawAdc = analogRead(BATTERY_PIN);
        int currentRSSI = WiFi.RSSI(); 
        
        if (xSemaphoreTake(xDataMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
            globalBatteryVoltage = (rawAdc / 4095.0) * 3.3 * 3.0; 
            globalWiFiRSSI = currentRSSI; 
            xSemaphoreGive(xDataMutex);
        }
        vTaskDelay(pdMS_TO_TICKS(10)); 
    }
}

void initMotors() {
    pinMode(PWMA, OUTPUT); pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT);
    pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT); pinMode(PWMB, OUTPUT);
    pinMode(STBY_PIN, OUTPUT); 
    digitalWrite(STBY_PIN, LOW); 
}

void moveMotors(int leftSpeed, int rightSpeed) {
    leftSpeed = constrain(leftSpeed, -255, 255);
    rightSpeed = constrain(rightSpeed, -255, 255);

    digitalWrite(AIN1, leftSpeed >= 0 ? LOW : HIGH); 
    digitalWrite(AIN2, leftSpeed >= 0 ? HIGH : LOW); 
    analogWrite(PWMA, abs(leftSpeed));

    digitalWrite(BIN1, rightSpeed >= 0 ? LOW : HIGH); 
    digitalWrite(BIN2, rightSpeed >= 0 ? HIGH : LOW); 
    analogWrite(PWMB, abs(rightSpeed));
}
