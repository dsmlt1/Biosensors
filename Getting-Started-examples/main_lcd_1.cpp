#include <LiquidCrystal.h>

// Define the LCD pins
const int rs = 27; // Register Select pin
const int en = 26; // Enable pin
const int d4 = 25; // Data pin 4
const int d5 = 33; // Data pin 5
const int d6 = 32; // Data pin 6
const int d7 = 35; // Data pin 7

// Initialize the LCD object
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

int counter = 0; // Counter variable

void setup()
{
    // Initialize serial communication
    Serial.begin(115200);

    // Initialize the LCD
    lcd.begin(16, 2); // Set the LCD dimensions (16 columns, 2 rows)
}

void loop()
{
    // Increment the counter
    counter++;

    // Clear the LCD screen
    lcd.clear();

    // Print the text and counter to the first row
    lcd.setCursor(0, 0); // Set cursor to the beginning of the first row
    lcd.print("Counter: ");
    lcd.print(counter);

    // Print a message to the second row
    lcd.setCursor(0, 1); // Set cursor to the beginning of the second row
    lcd.print("Hello, World!");

    // Delay for a short time
    delay(1000);
}