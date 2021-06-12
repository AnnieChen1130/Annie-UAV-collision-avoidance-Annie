#pragma once
#include <stdint.h>
struct Position {
	//Initialized to 0 to see if it has been initialized before. I'm using 0 as null value.
	Position() : Longitude(0), Latitude(0), Altitude(0), timestamp(null) {};
	fpos64_t Latitude;
	fpos64_t Longitude;
	fpos64_t Altitude;

	uint8_t timestamp;
};
