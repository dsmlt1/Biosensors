#include <Arduino.h>
#include <driver/adc.h>

// Pin configuration
#define HEARTRATE_SENSOR_PIN 34 // Analog input pin for ESP32 (ADC1_CH6)
#define LED_BUILTIN 16          // Built-in LED pin
#define SENSOR_LED_PIN 13       // Digital pin to control sensor's LED

// Constants for heart rate calculation
#define SAMPLE_RATE 20
#define WINDOW_SIZE 500
#define THRESHOLD_PERCENTAGE 65

class HeartRateMonitor
{
private:
    int samples[WINDOW_SIZE];
    int sampleIndex = 0;
    unsigned long lastSampleTime = 0;
    int threshold;
    bool isPeak = false;
    unsigned long lastPeakTime = 0;
    int beatCount = 0;
    unsigned long measurementStartTime = 0;

    int calculateThreshold()
    {
        int max = 0;
        int min = 4095;

        for (int i = 0; i < WINDOW_SIZE; i++)
        {
            if (samples[i] > max)
                max = samples[i];
            if (samples[i] < min && samples[i] > 0)
                min = samples[i];
        }

        Serial.print("Max: ");
        Serial.print(max);
        Serial.print(" Min: ");
        Serial.print(min);
        int calculatedThreshold = min + ((max - min) * THRESHOLD_PERCENTAGE / 100);
        Serial.print(" Calculated Threshold: ");
        Serial.println(calculatedThreshold);

        return calculatedThreshold;
    }

public:
    HeartRateMonitor()
    {
        memset(samples, 0, sizeof(samples));
    }

    void begin()
    {
        // Configure ADC
        adc1_config_width(ADC_WIDTH_BIT_12);
        adc1_config_channel_atten(ADC1_CHANNEL_6, ADC_ATTEN_DB_11);

        pinMode(HEARTRATE_SENSOR_PIN, INPUT);
        pinMode(LED_BUILTIN, OUTPUT);
        pinMode(SENSOR_LED_PIN, OUTPUT);

        // Test both LEDs at startup
        digitalWrite(LED_BUILTIN, HIGH);
        digitalWrite(SENSOR_LED_PIN, HIGH);
        delay(1000);
        digitalWrite(LED_BUILTIN, LOW);
        digitalWrite(SENSOR_LED_PIN, LOW);
        Serial.println("LED test completed");

        // Initial threshold calculation
        for (int i = 0; i < WINDOW_SIZE; i++)
        {
            samples[i] = analogRead(HEARTRATE_SENSOR_PIN);
            delay(5);
        }
        threshold = calculateThreshold();

        measurementStartTime = millis();
    }

    int update()
    {
        unsigned long currentTime = millis();

        if (currentTime - lastSampleTime >= (1000 / SAMPLE_RATE))
        {
            lastSampleTime = currentTime;

            int value = analogRead(HEARTRATE_SENSOR_PIN);
            samples[sampleIndex] = value;
            sampleIndex = (sampleIndex + 1) % WINDOW_SIZE;

            if (sampleIndex % 50 == 0)
            {
                Serial.print("Analog Value: ");
                Serial.print(value);
                Serial.print(" Threshold: ");
                Serial.println(threshold);
            }

            if (sampleIndex % 100 == 0)
            {
                threshold = calculateThreshold();
            }

            // Update both LEDs based on peak detection
            if (value > threshold && value < 4090 && !isPeak)
            {
                isPeak = true;
                digitalWrite(LED_BUILTIN, HIGH);
                digitalWrite(SENSOR_LED_PIN, HIGH);
                Serial.println("Peak detected - LEDs ON");

                if (lastPeakTime > 0)
                {
                    unsigned long peakInterval = currentTime - lastPeakTime;
                    if (peakInterval >= 300 && peakInterval <= 2000)
                    {
                        beatCount++;
                    }
                }
                lastPeakTime = currentTime;
            }
            else if (value < threshold || value >= 4090)
            {
                isPeak = false;
                digitalWrite(LED_BUILTIN, LOW);
                digitalWrite(SENSOR_LED_PIN, LOW);
            }
        }

        if (currentTime - measurementStartTime >= 10000)
        {
            int bpm = (beatCount * 6);
            beatCount = 0;
            measurementStartTime = currentTime;
            return bpm;
        }

        return -1;
    }
};

HeartRateMonitor heartRateMonitor;

void setup()
{
    Serial.begin(9600);
    while (!Serial)
    {
        ; // Wait for Serial to be ready
    }
    Serial.println("Starting Heart Rate Monitor...");

    // Test both LEDs independently
    pinMode(LED_BUILTIN, OUTPUT);
    pinMode(SENSOR_LED_PIN, OUTPUT);
    Serial.println("Testing LEDs...");

    // Test built-in LED
    digitalWrite(LED_BUILTIN, HIGH);
    delay(1000);
    digitalWrite(LED_BUILTIN, LOW);

    // Test sensor LED
    digitalWrite(SENSOR_LED_PIN, HIGH);
    delay(1000);
    digitalWrite(SENSOR_LED_PIN, LOW);

    heartRateMonitor.begin();
    Serial.println("Heart Rate Monitor initialized");
}

void loop()
{
    int bpm = heartRateMonitor.update();

    if (bpm > 0)
    {
        Serial.print("Heart Rate: ");
        Serial.print(bpm);
        Serial.println(" BPM");
    }

    delay(10);
}