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

// Function to update LED status based on system state
void updateLEDStatus() {
    digitalWrite(LED_GPS_PIN, HIGH); // Turn on GPS LED
    digitalWrite(LED_SD_PIN, HIGH); // Turn on SD card LED
    digitalWrite(LED_ERROR_PIN, LOW); // Turn off error LED
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
    lora.print(data);
    delay(1000); // Adjust delay as needed for LoRa transmission
}

// Function to control camera module start
void startCamera() {
    digitalWrite(CAMERA_CONTROL_PIN, HIGH);
}

// Function to control camera module stop
void stopCamera() {
    digitalWrite(CAMERA_CONTROL_PIN, LOW);
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
