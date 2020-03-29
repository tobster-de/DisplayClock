#include <DS1307.h>
#include <Wire.h>
#include <LiquidCrystal_PCF8574.h>

//#define RTC_I2C_ADDR        0x68
#define DISPLAY_I2C_ADDR    0x3F

#define RTC_SQW_PIN         2       // pin 2 (square wave generator -> interrupt)

LiquidCrystal_PCF8574 LCD(DISPLAY_I2C_ADDR);

byte UChars[8][8] =
{ {0, 0, 0, 0, 1, 3, 7, 15},
  {0, 0, 0, 0, 16, 24, 28, 30},
  {15, 7, 3, 1, 0, 0, 0, 0},
  {30, 28, 24, 16, 0, 0, 0, 0},
  {0, 0, 0, 0, 31, 31, 31, 31},
  {31, 31, 31, 31, 0, 0, 0, 0},
  {31, 31, 31, 31, 30, 28, 24, 16},
  {16, 24, 28, 30, 31, 31, 31, 31} };

char Digits[11][4][3] =
{ {{0,4,1}, {255,32,255}, {255,32,255}, {2,5,3}},
  {{0,4,32}, {32,255,32}, {32,255,32}, {5,5,5}},
  {{0,4,1}, {0,4,6}, {255,32,32}, {5,5,5}},
  {{0,4,1}, {32,0,6}, {32,2,7}, {2,5,3}},
  {{4,32,32}, {255,4,255}, {32,32,255}, {32,32,5}},
  {{4,4,4}, {255,4,1}, {32,32,255}, {2,5,3}},
  {{0,4,1}, {255,4,1}, {255,32,255}, {2,5,3}},
  {{4,4,4}, {32,0,6}, {32,255,32}, {32,5,32}},
  {{0,4,1}, {255,4,255}, {255,32,255}, {2,5,3}},
  {{0,4,1}, {255,4,255}, {32,32,255}, {2,5,3}},
  {{0,4,1}, {255,32,255}, {255,32,255}, {2,5,3}} };

//BinChars: array[0..2] of TCharPattern =
// ((0, 0, 0, 0, 0, 14, 14, 14), (14, 14, 14, 0, 0, 0, 0, 0), (14, 14, 14, 0, 0, 14, 14, 14));

void setup()
{
    int error;

    Serial.begin(115200);
    Serial.println("LCD...");

    while (!Serial);

    Serial.println("... check for LCD");

    // See http://playground.arduino.cc/Main/I2cScanner
    Wire.begin();
    Wire.beginTransmission(DISPLAY_I2C_ADDR);
    error = Wire.endTransmission();

    if (error == 0)
    {
        Serial.println(": LCD found.");
    }
    else
    {
        Serial.println(": LCD not found.");

        Serial.print("Error: ");
        Serial.println(error);
    }

    LCD.begin(20, 4); // initialize the lcd

    //  setup user characters
    for (byte i = 0; i < 8; i++)
    {
        LCD.createChar(i, UChars[i]);
    }

    LCD.clear();

    PrintDigit(0, 1);

    // 1Hz square wave (every second)
    //RTC.SetOutput(DS1307_SQW1HZ);
    //RTC.start();
}

void PrintDigit(byte col, byte digit)
{
    for (byte row = 0; row < 4; row++)
    {
        LCD.setCursor(col, row);
        for (byte c = 0; c < 3; c++)
        {
            LCD.print(Digits[digit][row][c]);
        }
    }
}

void loop()
{


}
