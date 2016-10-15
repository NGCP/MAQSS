/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   flight_logger.hpp
 * Author: zhangh94
 *
 * Created on September 30, 2016, 6:27 PM
 */

#ifndef FLIGHT_LOGGER_HPP
#define FLIGHT_LOGGER_HPP

#include <iostream>
#include <fstream>
#include <string>
#include <chrono>

#include <mavlink/common/mavlink.h>
#include <autopilot_interface.h>

using namespace std::chrono;
class flight_logger {
public:
    flight_logger();
    void log(Mavlink_Messages *msgs);
    virtual ~flight_logger();
private:
    std::ofstream flt_log;
    std::string filename;
    struct tm *now; // time when logger was started
    unsigned int ndx = 0;
    steady_clock::time_point t, t0;
    bool firstLog = true;
};

#endif /* FLIGHT_LOGGER_HPP */

