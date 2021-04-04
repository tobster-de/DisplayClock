#include "DaylightSavingsTime.h"

/*
// https://www.mikrocontroller.net/topic/144905
// https://www.math.uni-bielefeld.de/~sillke/ALGORITHMS/calendar/weekday.c
int getDayOfWeek(time_t time)
{
    int tYear = year(time);
    uint8_t tDay = day(time);
    uint8_t tMonth = month(time);

    // Variation of Sillke for the Gregorian calendar.
    // With 0=Sunday, 1=Monday, ... 6=Saturday
    if ((tMonth -= 2) <= 0) {
        tMonth += 12;
        tYear--;
    }

    return (83 * tMonth / 32 + tDay + tYear + tYear / 4 - tYear / 100 + tYear / 400) % 7;
}
*/

// https://forum.arduino.cc/index.php?topic=172044.0
bool isDayLightSavingsTime_EU(time_t t, byte tzHours)
{
    int tYear = year(t);
    uint8_t tMonth = month(t);
    uint8_t tDay = day(t);
    uint8_t tHour = hour(t);

    if (tMonth < 3 || tMonth > 10) return false;  // keine Sommerzeit in Jan, Feb, Nov, Dez
    if (tMonth > 3 && tMonth < 10) return true;   // Sommerzeit in Apr, Mai, Jun, Jul, Aug, Sep

    if (tMonth == 3 && (tHour + 24 * tDay) >= (1 + tzHours + 24 * (31 - (5 * tYear / 4 + 4) % 7))
        || tMonth == 10 && (tHour + 24 * tDay) < (1 + tzHours + 24 * (31 - (5 * tYear / 4 + 1) % 7)))
    {
        return true;
    }

    return false;
}