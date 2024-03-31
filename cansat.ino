// Include necessary libraries
#include <Wire.h>
#include <MPU6050.h>
#include <SD.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <DS3231.h> // RTC library
#include <SoftwareSerial.h> // Software serial library for SIM808 GPS module

// Define sensor pins and addresses
#define MPU_ADDR 0x68 // MPU6050 address
#define LED_GPS_PIN 25 // LED pin for GPS status
#define LED_SD_PIN 26  // LED pin for SD card write status
#define LED_ERROR_PIN 27 // LED pin for error indication
#define SIM808_RX_PIN 0 // Connected to TX of SIM808
#define SIM808_TX_PIN 1 // Connected to RX of SIM808
#define ACS712_PIN 13 // Analog pin for ACS712 current sensor
#define VOLTAGE_PIN 12 // Analog pin for voltage divider
#define UV_SENSOR_PIN 15 // Analog pin for UV sensor
#define LORA_RX_PIN 17 // LoRa module RX pin (GPIO17)
#define LORA_TX_PIN 16 // LoRa module TX pin (GPIO16)
#define CAMERA_TX_PIN 22 // Camera module TX pin (GPIO22)
#define CAMERA_RX_PIN 21 // Camera module RX pin (GPIO21)
#define CAMERA_CONTROL_PIN 4 // Camera module control pin (GPIO4)

// Define sensor objects
MPU6050 mpu(Wire);
HardwareSerial sim808(2); // Using hardware serial port 2 for SIM808 module
Adafruit_BME280 bme;
DS3231 rtc(21, 22); // SDA, SCL for I2C communication (GPIO21, GPIO22)
SoftwareSerial sim808GPS(SIM808_RX_PIN, SIM808_TX_PIN); // Software serial for SIM808 GPS module

// Define file object for SD card
File dataFile;

// Define phone number for SMS notifications
String phoneNumber = "1234567890";

void setup() {
    // Initialize serial communication
    Serial.begin(9600);
    while (!Serial) {
        ; // Wait for serial port to connect
    }

    // Initialize LED pins
    pinMode(LED_GPS_PIN, OUTPUT);
    pinMode(LED_SD_PIN, OUTPUT);
    pinMode(LED_ERROR_PIN, OUTPUT);
    digitalWrite(LED_GPS_PIN, LOW); // Ensure LEDs are initially off
    digitalWrite(LED_SD_PIN, LOW);
    digitalWrite(LED_ERROR_PIN, LOW);

    // Initialize SD card
    if (!SD.begin(5)) {  // SD card module connected to pin 5 (GPIO5)
        blinkError(); // Blink LED to indicate SD card initialization failure
        Serial.println("SD Card initialization failed!");
        return;
    }

    // Initialize I2C communication and sensors
    Wire.begin(21, 22); // Start I2C communication with SDA on GPIO21 and SCL on GPIO22
    if (!mpu.begin()) {
        blinkError(); // Blink LED to indicate MPU6050 initialization failure
        Serial.println("Could not find a valid MPU6050 sensor, check wiring!");
        while (1);
    }

    if (!bme.begin(0x76)) {
        blinkError(); // Blink LED to indicate BME280 initialization failure
        Serial.println("Could not find a valid BME280 sensor, check wiring!");
        while (1);
    }

    // Initialize SIM808 module
    sim808.begin(9600, SERIAL_8N1, SIM808_RX_PIN, SIM808_TX_PIN);
    sim808GPS.begin(9600); // Start serial communication with SIM808 GPS module
    rtc.begin();

    sendATCommand("AT"); // Check if module is responsive
    sendATCommand("AT+CMGF=1"); // Set SMS mode to text mode

    // Initialize camera control pin
    pinMode(CAMERA_CONTROL_PIN, OUTPUT);
}

void loop() {
    // Read sensor data and perform telemetry transmission
    readAndTransmitSensorData();

    // Listen for commands from LoRa module and control camera accordingly
    if (Serial.available() > 0) {
        String command = Serial.readStringUntil('\n');
        if (command == "start") {
            startCamera();
        } else if (command == "stop") {
            stopCamera();
        }
    }

    // Update GPS data
    while (sim808GPS.available() > 0) {
        if (sim808GPS.find("$GPRMC")) {
            // Process GPS data
            updateGPSData();
        }
    }
}

// Function to read sensor data and perform telemetry transmission
void readAndTransmitSensorData() {
    // Read sensor data
    float temperature =
    char *token = strtok(dataCharArray, ",");
    int index = 0;

    while (token != NULL) {
        if (index == 3) {
            // Latitude
            latitude = atof(token);
        } else if (index == 5) {
            // Longitude
            longitude = atof(token);
        } else if (index == 7) {
            // Velocity
            velocity = atof(token);
        }
        token = strtok(NULL, ",");
        index++;
    }
}
