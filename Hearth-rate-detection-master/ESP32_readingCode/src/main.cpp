#include <Arduino.h>

#define LED 2

// Potentiometer is connected to GPIO 32 (Analog ADC1_CH6)
unsigned long previousMillis = 0;
const long interval = 2.76; // sampling rate 360Hz
int sensorValue;
float voltage;
float buffer[720]={0};
int i=0;


void setup() {
  Serial.begin(9600);
  
}

void loop() {
  unsigned long currentMillis = millis();
  
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    sensorValue = analogRead(A4);
    voltage = sensorValue * (3.3 / 4095.0);
    buffer[i++] = voltage;
  }

  if (i==720){
    i=0;
    for (float x : buffer) // for each element 'x' in the array 'X'
    {
    Serial.println(x);
    }     
  }
}
