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
#include <Eigen/Dense>

using namespace Eigen;
class configContainer {
public:
    configContainer() {}
    virtual ~configContainer() {};
    
    // Config parameters
    std::string version;
    std::string pipe_CV_to_PNav; // Named pipe for CV to write to PNAV and PNAV to read from CV
    std::string pipe_PNav_to_CV; // Named pipe for PNAV to write to CV and CV to read from PNAV
    std::string input_filename;
    std::string uart_name;
    int quad_id; // quad ID (A=0, B=1, C=2)
    bool log = false; // log flight data
    float log_freq = 2; // log frequency in Hz
    int debug_delay = 0; // delay at each loop of main loop for debugging purposes
    int baudrate = 57600; // microcomputer to flightcontroller baudrate (57600 default)
    int fd_CV_to_PNav;
    int fd_PNav_to_CV;
    float heartbeat_freq = 0.5; // rate to send heartbeats to GCS[1/s]
    
    // Hardcoded Mission Parameters
    unsigned int npoints = 25; // number of waypoints to generate for flight pattern
    unsigned int pattern = 999; // flight pattern according to pattern enum in waypoints.hpp. Leave as 999 to let GCS specify
    int head = 999; // compass heading (deg) which can be set from InputFile. Leave as 999 to let GCS specify
    int dist = 0; // distance parameter for mission in meters. Leave as 0 to let GCS specify
    int alt = 7; // altitude to fly mission at [m]
    float field_heading = 139; // fieldHeading in degrees
    float setpoint_tolerance = 3.0; // distance in meters to reach to current setpoint before moving onto next setpoint [m]]
    
    // Camera Parameters
    unsigned int cam_Width = 1920; // image capture resolution (pixels)
    unsigned int cam_Height = 1080;
    int cam_FOV_v = 80; // camera vertical field of view angle (degrees)
    int cam_FOV_h = 90; // camera horizontal field of view angle (degrees)
    bool video_Stabilization = true; // sets raspicam videostabilization option
    
    bool cam_Test = false; // set camera to operate in camera test mode (capture numerous images)
    float cap_Freq=0.5; // picture capture rate for cam_Test mode in Hz
    
    // LocalNED parameters
    // TODO: Handle case if origin location gets corrupted
    bool originSet = false;
    Vector3d originECEF;
    Vector3d originLLA;
    Matrix3d Rne; // ECEF to LocalNED rotation matrix
    Matrix3d Rne_transpose; // LocalNED to ECEF rotation matrix
    
    std::map<std::string, void*> misc_Params; // store all other input params
private:

};

#endif /* CONFIGCONTAINER_HPP */

