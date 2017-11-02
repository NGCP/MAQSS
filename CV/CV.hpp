#ifndef CV_HPP
#define CV_HPP

#include "log.hpp"

#define MAX_IMGS 5000

// Camera settings
#define BRIGHTNESS 50.0f
#define CONTRAST 50.0f
#define SATURATION 50.0f
#define GAIN 50.0f

// Misc. definitions
#define START_STR "Start"
#define EXIT_STR "Exit"
#define DONE_STR "Done"
#define FOUND_STR "Found"

// CV definitions
#define CV_BLUE 0
#define CV_GREEN 1
#define CV_RED 2

//Localization Definitions
//#define SENSOR_WIDTH 3.76
//#define FOCAL_LENGTH 3.6
#define PIXEL_RATIO 0.00317567567567567567567567567568
#define MID_WIDTH 160
#define MID_HEIGHT 90

// Connected component definitions
#define CC_MIN_AREA 11000
#define CC_MAX_AREA 12200
#define POINT_TOLERANCE 10

class configContainer;

void CV_call_stop();
void frameLoop(unsigned int& nCaptures, configContainer *configs, Log &logger);
void testLoop(unsigned int& nCaptures, configContainer *configs);

#endif
