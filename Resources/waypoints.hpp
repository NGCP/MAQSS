/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   waypoints.hpp
 * Author: zhangh94
 *
 * Created on September 1, 2016, 12:26 PM
 */

#ifndef WAYPOINTS_HPP
#define WAYPOINTS_HPP
#include <iostream>
#include <cstring>
#include <cstdlib>
#include <vector>
#include <array>
#include <fstream>
#include <cmath>
#include <Eigen/Dense>

#include "configContainer.hpp"

#define sind(x) (sin((x) * M_PI / 180.0))
#define cosd(x) (cos((x) * M_PI / 180.0))

using namespace Eigen;

enum pattern {
    RECTANGLE,
    CIRCLE,
    SQUARE,
    FIG8,
    RACETRACK,
    RACETRACK_WIDE,
    CAM_ALTITUDE_TEST,

    NUM_PATTERNS
};

typedef Vector3d coordLLA; // lat/long/alt in radians and meters
typedef Vector3d coordECEF; // XYZ in meters 
typedef Vector3d coordLocalNED; // NED in meters

// WGS84 Parameters (constants for coordinate transforms)
const unsigned int a = 6378137;
const unsigned int Rearth = 6371000; // [m]
const double b = 6356752.31424518;
const double a2 = pow(a, 2.0);
const double b2 = pow(b, 2.0);
const double f = 1 / 298.257223563;
const double e = sqrt((a2 - b2) / a2);
const double e2 = pow(e, 2.0);
const double ep = sqrt((a2 - b2) / b2);

class waypoints {
public:
    waypoints(configContainer *configs);
    void setWps(coordLocalNED startCoord, int heading, int length, float fieldHeading = 0.0, int pattern = RECTANGLE);

    void setCurrentWp(int Wp) {
        currentWp = Wp;
    }
    void plotWp();

    // static methods to convert between LLA and LocalNED. ConfigContiner is required because origin information is stored there
    static coordLocalNED LLAtoLocalNED(configContainer& configs, coordLLA &LLA);
    static coordLLA LocalNEDtoLLA(configContainer& configs, coordLocalNED &LNED);
    static coordLocalNED ECEFtoLocalNED(configContainer& configs, coordECEF &ECEF);
    static coordECEF LocalNEDtoECEF(configContainer& configs, coordLocalNED &LNED);
    static coordLLA ECEFtoLLA(coordECEF &ECEF);
    static coordECEF LLAtoECEF(coordLLA &LLA);
    static void findOriginLocalNED(configContainer& configs, coordLocalNED &LNED, coordLLA &LLA);
    static coordLLA coordAt(coordLLA &LLA, float bearing, float distance);

    std::vector<coordLocalNED> wps;

    virtual ~waypoints();
private:
    void calcFOV();
    int currentWp;
    int patternType;
    unsigned int npoints;
    float msnHeight; // mission altitude in LocalNED [m]
    float lv, lh; // Estimated frame capture size [m]]
    float FOV_V, FOV_H; // camera vertical and horizontal field of view angles [rads]

};

#endif /* WAYPOINTS_HPP */

