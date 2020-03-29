#include <DS1307.h>
#include <Wire.h>
#include <LiquidCrystal_PCF8574.h>

//#define SCAN_I2C

#define RTC_I2C_ADDR        0x68
#define DISPLAY_I2C_ADDR    0x38

#define STATUS_LED_PIN      13      // LED status
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

volatile boolean rtcFlag = false;
int readBuffer[16];
int lastHour = -1, lastMinute = -1, lastSecond = -1;

void setup()
{
    // configure Pins
    pinMode(STATUS_LED_PIN, OUTPUT);
    digitalWrite(STATUS_LED_PIN, LOW);
    pinMode(RTC_SQW_PIN, INPUT);
    digitalWrite(RTC_SQW_PIN, HIGH);

    Serial.begin(115200);
    while (!Serial);

    CheckI2C(DISPLAY_I2C_ADDR, "LCD");
    CheckI2C(RTC_I2C_ADDR, "RTC");

#ifdef SCAN_I2C
    ScanI2C();
#endif // SCAN_I2C

    // initialize the LCD
    LCD.begin(20, 4);
    LCD.setBacklight(148);

    //  setup user characters
    for (byte i = 0; i < 8; i++)
    {
        LCD.createChar(i, UChars[i]);
    }

    LCD.clear();
    PrintTime(0, 0, 0);

    Serial.println("To set time: send \"hh:mm:ss\"");

    // 1Hz square wave (every second)
    //RTC.start();
    RTC.SetOutput(DS1307_SQW1HZ);

    attachInterrupt(digitalPinToInterrupt(RTC_SQW_PIN), rtcTimerIsr, RISING);
}

void rtcTimerIsr()
{
    rtcFlag = true;
}

void CheckI2C(byte address, char* device)
{
    int error;

    Serial.print("Check for ");
    Serial.print(device);

    Wire.begin();
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    Serial.print(": ");
    Serial.print(device);

    if (error == 0)
    {
        Serial.println(" found.");
    }
    else
    {
        Serial.println(" not found.");

        Serial.print("Error: ");
        Serial.println(error);
    }
}

#ifdef SCAN_I2C
void ScanI2C()
{
    byte error, address;

    Serial.println("Scanning I2C...");

    for (address = 1; address < 127; address++)
    {
        // The i2c_scanner uses the return value of
        // the Write.endTransmisstion to see if
        // a device did acknowledge to the address.
        Wire.beginTransmission(address);
        error = Wire.endTransmission();

        if (error == 0)
        {
            Serial.print("I2C device found at address 0x");
            if (address < 16)
                Serial.print("0");
            Serial.println(address, HEX);
        }
        else if (error == 4)
        {
            Serial.print("Unknow error at address 0x");
            if (address < 16)
                Serial.print("0");
            Serial.println(address, HEX);
        }
    }
}
#endif // SCAN_I2C


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

void PrintTime(int hour, int minute, int second)
{
    static char string[10];

    if (hour != lastHour)
    {
        lastHour = hour;

        PrintDigit(1, hour / 10);
        PrintDigit(4, hour % 10);
    }

    if (minute != lastMinute)
    {
        lastMinute = minute;

        PrintDigit(10, minute / 10);
        PrintDigit(13, minute % 10);
    }

    if (second != lastSecond)
    {
        lastSecond = second;

        LCD.setCursor(17, 4);
        itoa(second % 60, string, 10);

        if (strlen(string) == 1)
        {
            LCD.print("0");
        }
        LCD.print(string);

        // blink seperator
        char sig = (second % 2 == 0) ? 'o' : (char)32;

        LCD.setCursor(8, 1);
        LCD.print(sig);
        LCD.setCursor(8, 2);
        LCD.print(sig);
    }
}

void loop()
{
    int hour, minute, second;

    // 1s RTC timer
    if (rtcFlag)
    {
        rtcFlag = false;
        hour = RTC.get(DS1307_HR, true);
        minute = RTC.get(DS1307_MIN, false);
        second = RTC.get(DS1307_SEC, false);

        /*
        Serial.print("Time: ");
        Serial.print(hour);
        Serial.print(":");
        Serial.print(minute);
        Serial.print(":");
        Serial.println(second);
        */

        PrintTime(hour, minute, second);

        // blink status LED
        digitalWrite(STATUS_LED_PIN, HIGH);
        delay(50);
        digitalWrite(STATUS_LED_PIN, LOW);
    }

    if (Serial.available() >= 8)
    {
        ReadSerial(readBuffer);

        Serial.print("Setting time: ");
        Serial.print(readBuffer[0]);
        Serial.print(":");
        Serial.print(readBuffer[1]);
        Serial.print(":");
        Serial.print(readBuffer[2]);
        Serial.println("");

        RTC.stop();

        RTC.set(DS1307_HR, readBuffer[0]);
        RTC.set(DS1307_MIN, readBuffer[1]);
        RTC.set(DS1307_SEC, readBuffer[2]);

        //RTC.set(DS1307_SEC, rtcBuffer[6]);
        //RTC.set(DS1307_MIN, rtcBuffer[5]);
        //RTC.set(DS1307_HR, rtcBuffer[4]);
        //RTC.set(DS1307_DOW, rtcBuffer[3]);
        //RTC.set(DS1307_DATE, rtcBuffer[2]);
        //RTC.set(DS1307_MTH, rtcBuffer[1]);
        //RTC.set(DS1307_YR, rtcBuffer[0]);
        RTC.start();
    }
}

void ReadSerial(int* values)
{
    static byte charIndex;
    static byte valueIndex;
    static char buffer[16];

    while (Serial.available())
    {
        char c = Serial.read();

        if (c == '\n')
        {
            buffer[charIndex] = '\0';
            values[valueIndex] = atoi(buffer);

            return;
        }
        else if (c >= 32)
        {
            if (c == ':' || c == 32)
            {
                buffer[charIndex] = '\0';
                values[valueIndex] = atoi(buffer);

                valueIndex++;
                charIndex = 0;
            }
            else if (charIndex < sizeof(buffer) - 1)
            {
                // buffer character
                buffer[charIndex++] = c;
            }
        }
    }
}