/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   configContainer.hpp
 * Author: zhangh94
 *
 * Created on September 12, 2016, 6:40 PM
 */

#ifndef CONFIGCONTAINER_HPP
#define CONFIGCONTAINER_HPP

#include <string>
#include <map>

class configContainer {
public:
    configContainer() {}
    virtual ~configContainer() {};
    
    // Config parameters
    std::string version;
    std::string cv_pipe; // Named pipe for CV to write to PNAV and PNAV to read from CV
    std::string cv_pipe2; // Named pipe for PNAV to write to CV and CV to read from PNAV
    std::string input_filename;
    std::string uart_name;
    int baudrate = 57600; // microcomputer to flightcontroller baudrate (57600 default)
    int cv_fd;
    int cv_fd2;
    unsigned int npoints = 25; // number of waypoints to generate for flight pattern
    unsigned int pattern = 999; // flight pattern according to pattern enum in waypoints.hpp. Leave as 999 to let GCS specify
    int head = 999; // compass heading (deg) which can be set from InputFile. Leave as 999 to let GCS specify
    int dist = 0; // distance parameter for mission in meters. Leave as 0 to let GCS specify
    int camFOV_v = 80; // camera vertical field of view angle (degrees)
    int camFOV_h = 90; // camera horizontal field of view angle (degrees)
    bool videoStabilization = true; // sets raspicam videostabilization option
    bool camTest = false; // set camera to operate in camera test mode (capture numerous images)
    
    std::map<std::string, void*> miscParams; // store all other input params
private:

};

#endif /* CONFIGCONTAINER_HPP */

