#include <Arduino.h>

void setup() {
    // Start serial communication
    Serial.begin(115200);
    // Wait a bit for the serial monitor to connect (optional)
    delay(2000);
    Serial.println("Blue Pill is alive!");
}

void loop() {
    // Print uptime in milliseconds
    Serial.print("Uptime: ");
    Serial.print(millis());
    Serial.println(" ms");

    // Wait one second
    delay(1000);
}
