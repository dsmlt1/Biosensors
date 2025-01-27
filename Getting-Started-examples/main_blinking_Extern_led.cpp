#include <Arduino.h>

#define INTERNAL_LED_PIN 2 // Built-in LED
#define EXTERNAL_LED_PIN 4

void setup()
{
    // Initialize both LED pins as outputs
    pinMode(INTERNAL_LED_PIN, OUTPUT);
    pinMode(EXTERNAL_LED_PIN, OUTPUT);

    // Start with LEDs off
    digitalWrite(INTERNAL_LED_PIN, LOW);
    digitalWrite(EXTERNAL_LED_PIN, LOW);
    Serial.begin(9600);
}

void loop()
{
    digitalWrite(INTERNAL_LED_PIN, HIGH);
    digitalWrite(EXTERNAL_LED_PIN, LOW);
    delay(1000);
    Serial.println("Hello, World JP");

    // Blink external LED
    digitalWrite(INTERNAL_LED_PIN, LOW);
    digitalWrite(EXTERNAL_LED_PIN, HIGH);
    delay(1000);
}
