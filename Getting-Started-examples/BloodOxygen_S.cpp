/*!
 * @file  gainHeartbeatSPO2.ino
 * @n experiment phenomena: get the heart rate and blood oxygenation, during the update the data obtained does not change
 * @copyright   Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
 * @license     The MIT License (MIT)
 * @author      PengKaixing(kaixing.peng@dfrobot.com)
 * @version     V1.0.0
 * @date        2021-06-21
 * @url         https://github.com/DFRobot/DFRobot_BloodOxygen_S
 */
#include <Wire.h>
#define I2C_COMMUNICATION
#include "DFRobot_BloodOxygen_S.h"

// #define I2C_COMMUNICATION // use I2C for communication, but use the serial port for communication if the line of codes were masked

#ifdef I2C_COMMUNICATION
#define I2C_ADDRESS 0x57
DFRobot_BloodOxygen_S_I2C MAX30102(&Wire, I2C_ADDRESS);
#else
/* ---------------------------------------------------------------------------------------------------------------
 *    board   |             MCU                | Leonardo/Mega2560/M0 |    UNO    | ESP8266 | ESP32 |  microbit  |
 *     VCC    |            3.3V/5V             |        VCC           |    VCC    |   VCC   |  VCC  |     X      |
 *     GND    |              GND               |        GND           |    GND    |   GND   |  GND  |     X      |
 *     RX     |              TX                |     Serial1 TX1      |     5     |   5/D6  |  D2   |     X      |
 *     TX     |              RX                |     Serial1 RX1      |     4     |   4/D7  |  D3   |     X      |
 * ---------------------------------------------------------------------------------------------------------------*/
#if defined(ARDUINO_AVR_UNO) || defined(ESP8266)
SoftwareSerial mySerial(4, 5);
DFRobot_BloodOxygen_S_SoftWareUart MAX30102(&mySerial, 9600);
#else
DFRobot_BloodOxygen_S_HardWareUart MAX30102(&Serial1, 9600);
#endif
#endif

void I2Cscan()
{
    byte error;
    byte address;

    Serial.println("I2C Scanner");
    for (address = 1; address < 127; address++)
    {
        // The Arduino two-wire interface requires that the address is shifted left
        // one bit when compared with the actual slave address
        Wire.beginTransmission(address << 1);
        error = Wire.endTransmission();

        if (error == 0)
        {
            Serial.print("I2C device found at address 0x");
            if (address < 16)
                Serial.print("0");
            Serial.print(address, HEX);
            Serial.println("  !");
        }
        else if (error == 4)
        {
            Serial.print("Unknown error at address 0x");
            if (address < 16)
                Serial.print("0");
            Serial.println(address, HEX);
        }
    }
    Serial.println("Done.");
}

void setup()
{
    Serial.begin(9600);
    Wire.begin(); // Initialize I2C communication

    // Run the I2C scanner
    // I2Cscan();
    Serial.print(MAX30102.begin());
    // while (false == MAX30102.begin())
    // {
    //   Serial.println("init fail!");
    //   delay(1000);
    // }
    if (MAX30102.begin())
    {
        Serial.println("init success!");
        Serial.println("start measuring...");
        MAX30102.sensorStartCollect();
    }
    else
    {
        Serial.println("init fail!");
        while (1)
            ; // Stop the program if initialization fails
    }
    Serial.println("init success!");
    Serial.println("start measuring...");
    MAX30102.sensorStartCollect();
}

void loop()
{
    MAX30102.getHeartbeatSPO2();
    Serial.print("SPO2 is : ");
    Serial.print(MAX30102._sHeartbeatSPO2.SPO2);
    Serial.println("%");
    Serial.print("heart rate is : ");
    Serial.print(MAX30102._sHeartbeatSPO2.Heartbeat);
    Serial.println("Times/min");
    Serial.print("Temperature value of the board is : ");
    Serial.print(MAX30102.getTemperature_C());
    Serial.println(" ℃");
    // The sensor updates the data every 4 seconds
    delay(4000);
    // Serial.println("stop measuring...");
    // MAX30102.sensorEndCollect();
}