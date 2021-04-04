#include <DS1307RTC.h>
#include <TimeLib.h>
#include <Wire.h>
#include <LiquidCrystal_PCF8574.h>
#include <SerialCommand.h>

#include "DaylightSavingsTime.h"

//#define SCAN_I2C

#define RTC_I2C_ADDR        0x68
#define DISPLAY_I2C_ADDR    0x38

#define DISPLAY_BRIGHTNESS_DAY      192     // brightness during day (6:00 - 19:00 )
#define DISPLAY_BRIGHTNESS_NIGHT     64     // brightness at night (19:00 - 6:00)

#define DISPLAY_LED_PIN     11      // LCD brightness PWM pin (drive a mosfet)
#define STATUS_LED_PIN      13      // LED status
#define RTC_SQW_PIN         2       // pin 2 (square wave generator -> interrupt)

LiquidCrystal_PCF8574 LCD(DISPLAY_I2C_ADDR);
SerialCommand SC;

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
int brightness = DISPLAY_BRIGHTNESS_DAY;
int timezone = 1;

void SetBrightness();

void CheckI2C(byte address, char* device);

void PrintTime();

void unrecognizedCommandHelp();
void timeCommand();
void dateCommand();

void rtcTimerIsr()
{
    rtcFlag = true;
}

void setup()
{
    // configure Pins
    pinMode(STATUS_LED_PIN, OUTPUT);
    digitalWrite(STATUS_LED_PIN, LOW);
    pinMode(RTC_SQW_PIN, INPUT);
    digitalWrite(RTC_SQW_PIN, HIGH);

    pinMode(DISPLAY_LED_PIN, OUTPUT);

    Serial.begin(115200);
    while (!Serial);

    CheckI2C(DISPLAY_I2C_ADDR, "LCD");
    CheckI2C(RTC_I2C_ADDR, "RTC");

#ifdef SCAN_I2C
    ScanI2C();
#endif // SCAN_I2C

    // initialize the LCD
    LCD.begin(20, 4);
    SetBrightness();

    //  setup user characters
    for (byte i = 0; i < 8; i++)
    {
        LCD.createChar(i, UChars[i]);
    }

    LCD.clear();

    // the function to get the time from the RTC
    setSyncProvider(syncRTC);
    if (timeStatus() != timeSet)
        Serial.println("Unable to sync with the RTC");
    else
        Serial.println("RTC has set the system time");

    PrintTime();

    SC.addCommand("time", timeCommand);                 // output/set time
    SC.addCommand("date", dateCommand);                 // output/set date
    SC.addDefaultHandler(unrecognizedCommandHelp);      // Handler for command that isn't matched

    unrecognizedCommandHelp();

    // 1Hz square wave (every second)
    RTC.setSqwOutput(sqw1Hz);

    attachInterrupt(digitalPinToInterrupt(RTC_SQW_PIN), rtcTimerIsr, RISING);
}

// This gets set as the default handler, and gets called when no other command matches. 
void unrecognizedCommandHelp()
{
    Serial.println("");
    Serial.println("Valid commands are:");
    Serial.println("  time [hh mm [ss=0] [tz=1]]:");
    Serial.println("    outputs current system time/date, sets time to hh:mm:ss - timezone=tz");
    Serial.println("  date [dd MM yyyy]:");
    Serial.println("    outputs current system time/date, sets date to dd:MM:yyyy");
}

void SerialOutputTime()
{
    time_t time = now() + SECS_PER_HOUR * timezone;
    bool isDST = isDayLightSavingsTime_EU(time, timezone);
    if (isDST)
    {
        time += SECS_PER_HOUR;
    }

    int tHour = hour(time);
    int tMinute = minute(time);
    int tSecond = second(time);

    Serial.print("Time: ");
    if (tHour < 10) Serial.print("0");
    Serial.print(tHour);
    Serial.print(":");
    if (tMinute < 10) Serial.print("0");
    Serial.print(tMinute);
    Serial.print(":");
    if (tSecond < 10) Serial.print("0");
    Serial.print(tSecond);
    Serial.print(", timezone ");
    if (timezone > 0) Serial.print("+");
    Serial.print(timezone);
    if (isDST) Serial.print(", DST");
    Serial.println("");

    int tDay = day(time);
    int tMonth = month(time);
    int tYear = year(time);

    Serial.print("Date: ");
    if (tDay < 10) Serial.print("0");
    Serial.print(tDay);
    Serial.print(".");
    if (tMonth < 10) Serial.print("0");
    Serial.print(tMonth);
    Serial.print(".");
    Serial.println(tYear);

    Serial.flush();
}

void timeCommand()
{
    int val[4];
    char *arg;
    TimeElements time_elements;
    boolean argsgood = true;

    breakTime(now(), time_elements);
    val[2] = 0;
    val[3] = 1;

    for (int i = 0; i < 4; i++)
    {
        arg = SC.next();
        if (arg != NULL)
        {
            val[i] = atoi(arg);
        }
        else if (i < 2)
        {
            argsgood = false;
        }
    }

    if (argsgood)
    {
        time_elements.Hour = val[0];
        time_elements.Minute = val[1];
        time_elements.Second = val[2];
        timezone = val[3];

        // to UTC
        time_t newtime = makeTime(time_elements) - SECS_PER_HOUR * timezone;
        bool isDST = isDayLightSavingsTime_EU(newtime, timezone);
        if (isDST)
        {
            newtime -= SECS_PER_HOUR;
        }

        RTC.set(newtime);   // set the RTC and the system time to the received value
        setTime(newtime);
    }

    SerialOutputTime();
}

void dateCommand()
{
    int val[3];
    char *arg;
    TimeElements time_elements;
    boolean argsgood = true;

    breakTime(now(), time_elements);

    for (int i = 0; i < 3; i++)
    {
        arg = SC.next();
        if (arg != NULL)
        {
            val[i] = atoi(arg);
        }
        else
        {
            argsgood = false;
        }
    }

    if (argsgood)
    {
        time_elements.Day = val[0];
        time_elements.Month = val[1];
        time_elements.Year = CalendarYrToTm(val[2]);

        time_t newtime = makeTime(time_elements);

        RTC.set(newtime);   // set the RTC and the system time to the received value
        setTime(newtime);
    }

    SerialOutputTime();
}

void CheckI2C(byte address, char* device)
{
    Serial.print("Check for ");
    Serial.print(device);

    Wire.begin();
    Wire.beginTransmission(address);
    int error = Wire.endTransmission();

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

void PrintTime()
{
    time_t time = now() + SECS_PER_HOUR * timezone;
    if (isDayLightSavingsTime_EU(time, timezone))
    {
        time += SECS_PER_HOUR;
    }

    int tHour = hour(time);
    int tMinute = minute(time);
    int tSecond = second(time);

    static char string[10];

    if (tHour != lastHour)
    {
        lastHour = tHour;

        PrintDigit(1, tHour / 10);
        PrintDigit(4, tHour % 10);
    }

    if (tMinute != lastMinute)
    {
        lastMinute = tMinute;

        PrintDigit(10, tMinute / 10);
        PrintDigit(13, tMinute % 10);
    }

    if (tSecond != lastSecond)
    {
        lastSecond = tSecond;

        LCD.setCursor(17, 4);
        itoa(tSecond % 60, string, 10);

        if (strlen(string) == 1)
        {
            LCD.print("0");
        }
        LCD.print(string);

        // blink seperator
        char sig = (tSecond % 2 == 0) ? 'o' : (char)32;

        LCD.setCursor(8, 1);
        LCD.print(sig);
        LCD.setCursor(8, 2);
        LCD.print(sig);
    }
}

void SetBrightness()
{
    //Serial.print("Brightness: ");
    //Serial.println(brightness & 0xFF);

    // turn completely on/off using the PCF IC
    LCD.setBacklight(brightness & 0xFF);

    // use PWM to drive a P-CH mosfet, hence the inverted value
    analogWrite(DISPLAY_LED_PIN, 0xFF - (brightness & 0xFF));
}

void loop()
{
    // 1s RTC timer
    if (rtcFlag)
    {
        rtcFlag = false;
        int tHour = hour();
        int tMinute = minute();
        int tSecond = second();

        PrintTime();

        if (tSecond == 0)
        {
            brightness = (tHour >= 19 || tHour <= 6) ? DISPLAY_BRIGHTNESS_NIGHT : DISPLAY_BRIGHTNESS_DAY;
            SetBrightness();
        }

        // blink status LED
        digitalWrite(STATUS_LED_PIN, HIGH);
        delay(50);
        digitalWrite(STATUS_LED_PIN, LOW);
    }

    SC.readSerial();    // process serial commands
}
