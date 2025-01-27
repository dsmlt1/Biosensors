// Include necessary libraries
#include <Arduino.h>
#include <Wire.h>
#include "MAX30105.h"
#include "heartRate.h"
#include "spo2_algorithm.h"

MAX30105 particleSensor;

// Variables for heart rate calculation
const byte RATE_SIZE = 4; // Increase this for more averaging
byte rates[RATE_SIZE];    // Array of heart rates
byte rateSpot = 0;
long lastBeat = 0; // Time at which the last beat occurred
float beatsPerMinute;
int beatAvg;

// Variables for SpO2 calculation
#define MAX_BRIGHTNESS 255
uint32_t irBuffer[100];     // infrared LED sensor data
uint32_t redBuffer[100];    // red LED sensor data
int32_t bufferLength = 100; // buffer length of 100 stores 4 seconds of samples running at 25sps
int32_t spo2;               // SPO2 value
int8_t validSPO2;           // indicator to show if the SPO2 calculation is valid
int32_t heartRate;          // heart rate value
int8_t validHeartRate;      // indicator to show if the heart rate calculation is valid

void setup()
{
    Serial.begin(9600);

    // Initialize sensor
    if (!particleSensor.begin(Wire, I2C_SPEED_FAST))
    {
        Serial.println("MAX30105 was not found. Please check wiring/power.");
        while (1)
            ;
    }
    Serial.println("MAX30105 was found");
    // Configure sensor settings
    particleSensor.setup(MAX_BRIGHTNESS, 4, 2, 411, 4096); // Configure sensor with these settings

    // 25 samples per second, LED pulseWidth=411μs, ADC range=4096
    particleSensor.setPulseAmplitudeRed(0x0A);
    particleSensor.setPulseAmplitudeGreen(0); // Turn off Green LED
    particleSensor.setPulseAmplitudeIR(0x0A);
}

void loop()
{
    // Read from sensor
    long irValue = particleSensor.getIR();
    long redValue = particleSensor.getRed();

    // Fill our buffers
    for (byte i = 0; i < bufferLength; i++)
    {
        while (particleSensor.available() == false)
            particleSensor.check(); // Check the sensor for new data

        redBuffer[i] = particleSensor.getRed();
        irBuffer[i] = particleSensor.getIR();
        particleSensor.nextSample(); // We're finished with this sample so move to next sample
    }

    // Calculate heart rate and SpO2
    maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);

    // Calculate traditional heart rate detection
    if (checkForBeat(irValue) == true)
    {
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

    // Print results to Serial plotter
    if (irValue > 50000)
    {
        Serial.print("HR:");
        Serial.print(beatAvg);
        Serial.print(",SPO2:");
        Serial.print(spo2);
        Serial.print(",IR:");
        Serial.print(irValue / 800); // Scale down IR value for visualization
        Serial.print(",RED:");
        Serial.println(redValue / 800); // Scale down Red value for visualization
    }
    else
    {
        Serial.println("No finger detected");
    }

    delay(10); // Add a small delay to prevent overwhelming the serial output
}