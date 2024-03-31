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
    float temperature = readTemperature();
    float humidity = readHumidity();
    float pressure = readPressure();
    float altitude = readAltitude();
    float voltage = readVoltage();
    float current = readCurrent();
    int uvIndex = readUVIndex();
    long timestamp = getUnixTimestamp(); // Get Unix timestamp

    // Format telemetry data
    String telemetryData = "Temperature: " + String(temperature) + "C, "
                         + "Humidity: " + String(humidity) + "%, "
                         + "Pressure: " + String(pressure) + "hPa, "
                         + "Altitude: " + String(altitude) + "m, "
                         + "Voltage: " + String(voltage) + "V, "
                         + "Current: " + String(current) + "A, "
                         + "UV Index: " + String(uvIndex) + ", "
                         + "GPS Coordinates: " + String(latitude, 6) + ", " + String(longitude, 6) + ", "
                         + "Velocity: " + String(velocity) + " km/h, "
                         + "Timestamp: " + String(timestamp);

    // Log telemetry data to SD card
    logData(telemetryData);

    // Send telemetry data via LoRa
    sendDataViaLoRa(telemetryData);
}

// Function to read temperature from BME280 sensor
float readTemperature() {
    return bme.readTemperature();
}

// Function to read humidity from BME280 sensor
float readHumidity() {
    return bme.readHumidity();
}

// Function to read pressure from BME280 sensor
float readPressure() {
    return bme.readPressure();
}

// Function to calculate altitude from pressure
float readAltitude() {
    return bme.readAltitude(1013.25); // Sea-level pressure for calibration
}

// Function to read voltage from voltage divider
float readVoltage() {
    int sensorValue = analogRead(VOLTAGE_PIN);
    float voltage = sensorValue * (5.0 / 1023.0); // Assuming 5V reference
    return voltage;
}

// Function to read current from ACS712 sensor
float readCurrent() {
    int sensorValue = analogRead(ACS712_PIN);
    // Convert analog value to current using calibration factor
    // For example, if 512 corresponds to 0 Amps and sensor has sensitivity of 100 mV/A, calibration factor is 0.1
    float current = (sensorValue - 512) * 0.1;
    return current;
}

// Function to read UV index from UV sensor
int readUVIndex() {
    int uvIndex = analogRead(UV_SENSOR_PIN); // Read analog value from UV sensor
    // Perform conversion to UV index based on sensor characteristics
    // Add your UV index calculation logic here
    return uvIndex;
}

// Function to get Unix timestamp
long getUnixTimestamp() {
    return rtc.getUnixTime();
}

// Function to update LED status based on system state
void updateLEDStatus() {
    // Add code to update LED status
}

// Function to send SMS using SIM808 module
void sendSMS(String message) {
    sendATCommand("AT+CMGS=\"" + phoneNumber + "\""); // Set recipient number
    sim808.print(message); // Send message
    sim808.write(26); // Send Ctrl+Z to terminate message
    delay(1000); // Wait for message to be sent
    while (sim808.available()) {
        Serial.write(sim808.read());
    }
}

// Function to log data to SD card
void logData(String data) {
    dataFile = SD.open("data.txt", FILE_WRITE);
    if (dataFile) {
        dataFile.println(data);
        dataFile.close();
        digitalWrite(LED_SD_PIN, HIGH); // Turn on SD card write LED
        delay(1000); // Keep LED on for 1 second
        digitalWrite(LED_SD_PIN, LOW); // Turn off SD card write LED
    } else {
        Serial.println("Error opening file for writing");
    }
}

// Function to send data via LoRa
void sendDataViaLoRa(String data) {
    // Add LoRa transmission code here
}

// Function to control camera module start
void startCamera() {
    // Add camera start code here
}

// Function to control camera module stop
void stopCamera() {
    // Add camera stop code here
}

// Function to blink error LED
void blinkError() {
    for (int i = 0; i < 3; i++) {
        digitalWrite(LED_ERROR_PIN, HIGH);
        delay(500);
        digitalWrite(LED_ERROR_PIN, LOW);
        delay(500);
    }
}

// Function to send AT command to SIM808 module
void sendATCommand(String command) {
    sim808.println(command);
    delay(1000);
    while (sim808.available()) {
        Serial.write(sim808.read());
    }
}

// Variables to store GPS data
float latitude = 0.0;
float longitude = 0.0;
float velocity = 0.0;

// Function to update GPS data
void updateGPSData() {
    // Read and parse GPS data
    String data = sim808GPS.readStringUntil('\n');
    char dataCharArray[data.length() + 1];
    strcpy(dataCharArray, data.c_str());

    // Tokenize data
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
