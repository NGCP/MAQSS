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
#include "processInterface.hpp"
enum pattern {
    RECTANGLE,
    CIRCLE,
    SQUARE,
    FIG8,
    RACETRACK,
    RACETRACK_WIDE,
    
    NUM_PATTERNS
    
};

// TODO: is coord a vector with 3 elements here?
typedef std::array<double, 3> coord;

#define sind(x) (sin((x) * M_PI / 180.0))
#define cosd(x) (cos((x) * M_PI / 180.0))
class waypoints {
public:
    waypoints(configContainer *configs);
    void setWps(coord startCoord, double heading, double length, int pattern=RECTANGLE);
    void setCurrentWp(int Wp) {currentWp = Wp;}
    void plotWp();
        
    std::vector<coord> wps;

    virtual ~waypoints();
private:
    void calcFOV();
    int currentWp;
    int patternType;
    unsigned int npoints;
    float msnHeight; // whatever units local NED is in [m]
    float lv, lh; // same units as msnHeight
    float FOV_V, FOV_H; // camera vertical and horizontal field of view angles [rads]

};

#endif /* WAYPOINTS_HPP */

