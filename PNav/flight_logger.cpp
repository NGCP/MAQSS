/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   flight_logger.cpp
 * Author: zhangh94
 * 
 * Created on September 30, 2016, 6:27 PM
 */

#include "flight_logger.hpp"

#include <ctime>


flight_logger::flight_logger() {

    time_t t = time(0); // get time now
    now = localtime(& t);

    // open new file with name flt_log_time_date
    filename = "flt_log_" + std::to_string(now->tm_hour + 1) + ":" + std::to_string(now->tm_min) + "_" 
            + std::to_string(now->tm_mon + 1) + "_" + std::to_string(now->tm_mday);
    flt_log.open(filename);

    flt_log << "Flight Log" << std::endl;
    flt_log << "Index,\tTime [ms],\tLocal Position X [m],\tLocal Position Y,\tLocal Position Z,\t" <<
            "Local Target X [m],\tLocal TargetY,\tLocal Target Z,\t" <<
            "Global Position Lat,\tGlobal Position Lon,\t,Alt [m],\tHeading [deg]" << std::endl;
}

void flight_logger::log(Mavlink_Messages *msgs) {

    if (firstLog) {
        // store an initial time
        firstLog = false;
        t0 = steady_clock::now(); 
    }
    
    // TODO: MAKE THIS LOOK NICER
    t = steady_clock::now();
    // Figure out how to put time in there
    flt_log << ndx++ << ", \t" << duration_cast<milliseconds>(t - t0).count() << ", \t" <<msgs ->local_position_ned.x << ",\t" << msgs->local_position_ned.y << ",\t" << msgs->local_position_ned.z << ",\t" << 
            msgs->position_target_local_ned.x << ",\t" << msgs->position_target_local_ned.y << ",\t" <<  msgs->position_target_local_ned.z << ",\t" << 
            msgs->global_position_int.lat << ",\t" << msgs->global_position_int.lon << ",\t" <<  
            msgs->global_position_int.alt << ",\t" <<  (float)(msgs->global_position_int.hdg)/100  << std::endl;
}

flight_logger::~flight_logger() {
    flt_log.close();
}

