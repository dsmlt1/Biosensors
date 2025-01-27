#include <Wire.h>
#include "MAX30105.h" // This library works for MAX30102 as well
#include "heartRate.h"

MAX30105 particleSensor;

// Define constants for heartbeat detection
const byte RATE_SIZE = 4; // Increase this for more averaging. 4 is good.
byte rates[RATE_SIZE];    // Array of heart rates
byte rateSpot = 0;
long lastBeat = 0; // Time at which the last beat occurred
float beatsPerMinute;
int beatAvg;

void setup()
{
    Serial.begin(115200);
    Serial.println("Initializing...");

    // Initialize sensor
    if (!particleSensor.begin(Wire, I2C_SPEED_FAST))
    {
        Serial.println("MAX30102 was not found. Please check wiring/power.");
        while (1)
            ;
    }
    Serial.println("Place your index finger on the sensor with steady pressure.");

    particleSensor.setup();                    // Configure sensor. Use 6.4mA for LED drive
    particleSensor.setPulseAmplitudeRed(0x0A); // Turn Red LED to low to indicate sensor is running
    particleSensor.setPulseAmplitudeGreen(0);  // Turn off Green LED
}

void loop()
{
    long irValue = particleSensor.getIR();

    if (checkForBeat(irValue) == true)
    {
        // We sensed a beat!
        long delta = millis() - lastBeat;
        lastBeat = millis();

        beatsPerMinute = 60 / (delta / 1000.0);

        if (beatsPerMinute < 255 && beatsPerMinute > 20)
        {
            rates[rateSpot++] = (byte)beatsPerMinute; // Store this reading in the array
            rateSpot %= RATE_SIZE;                    // Wrap variable

            // Take average of readings
            beatAvg = 0;
            for (byte x = 0; x < RATE_SIZE; x++)
                beatAvg += rates[x];
            beatAvg /= RATE_SIZE;
        }
    }

    Serial.print("IR=");
    Serial.print(irValue);
    Serial.print(", BPM=");
    Serial.print(beatsPerMinute);
    Serial.print(", Avg BPM=");
    Serial.println(beatAvg);

    delay(20);
}

// Function to check if a beat is detected
bool checkForBeat(long irValue)
{
    static long lastMax = 0;
    static long lastMin = 0;
    static boolean pulse = false;
    static boolean firstBeat = true;

    if (irValue > lastMax)
    {
        lastMax = irValue;
        pulse = true;
    }
    if (irValue < lastMin)
    {
        lastMin = irValue;
        if (pulse == true)
        {
            pulse = false;
            if (firstBeat)
            {
                firstBeat = false;
                return true;
            }
            if (lastMax - lastMin > 50000)
            {
                return true;
            }
        }
    }
    return false;
}