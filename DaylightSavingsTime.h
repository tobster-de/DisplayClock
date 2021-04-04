// time.h

#ifndef _DAYLIGHTSAVINGSTIME_h
#define _DAYLIGHTSAVINGSTIME_h

#include "TimeLib.h"

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

//int getDayOfWeek(time_t t);
bool isDayLightSavingsTime_EU(time_t t, byte tzHours);

#endif

