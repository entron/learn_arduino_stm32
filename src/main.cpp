#include <Arduino.h>
#include <HardwareSerial.h>

// Manually instantiate Serial2 on USART2 (PA2=TX, PA3=RX) if not provided by core
#if !defined(SERIAL_PORT_HARDWARE2) && !defined(Serial2)
HardwareSerial Serial2(USART2);
#endif

void setup() {
    // Start serial communication
    Serial2.begin(115200);
    // Wait a bit for the serial monitor to connect (optional)
    delay(2000);
    Serial2.println("Blue Pill is alive!");
}

void loop() {
    // Print uptime in milliseconds
    Serial2.print("Uptime: ");
    Serial2.print(millis());
    Serial2.println(" ms");

    // Wait one second
    delay(1000);
}
