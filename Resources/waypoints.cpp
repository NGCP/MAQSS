/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   waypoints.cpp
 * Author: zhangh94
 * 
 * Created on September 1, 2016, 12:26 PM
 */

#include <iostream>
#include <cstring>
#include <cstdlib>
#include <vector>
#include <array>
#include <fstream>
#include <cmath>

#include "waypoints.hpp"


waypoints::waypoints(configContainer *configs) {
    FOV_V = configs->camFOV_v/57.3;
    FOV_H = configs->camFOV_h/57.3;
    npoints = configs->npoints;
}

void waypoints::calcFOV()
{
    // Assuming theta = 0, camera pointed straight down
    float alpha = FOV_V/2.0; // rads
    float beta = (180/57.3 - FOV_H)/2.0;
    float lc = msnHeight; 
    float le = msnHeight/cos(-alpha);
    lv = 2 * sqrt(lc * lc + le * le - 2 *le * lc * cos(alpha));
    lh = (lc/(sin(beta)*sin(beta))) * sin(FOV_H);
    
}

void waypoints::setWps(coord startCoord, double heading, double length, int pattern)
{
    msnHeight = abs(startCoord[2]); // assume local NED z = 0 is ground
    calcFOV();
    currentWp = 0; // restart from Wp 0 when new Search Chunk is sent
    wps.clear();
    
    // RECTANGLE Variables
    float dx(0.0), dy(0.0), sgn(0.0);  
    float sinHeading = sind(heading);
    float cosHeading = cosd(heading);
    coord tmp;
    
    //FIG8 Variables
    unsigned int ndx(0);
    float t(0);
    float theta(0);
    float util(0);
    
    switch(pattern) 
    {
        case RECTANGLE:
            // Throw this into function so I can create variables inside
            wps.push_back(startCoord);
            
            // TODO: Check this works for all quadrants
            while (dx < (length * sinHeading))
            {
                sgn = 1 - sgn;
                dy = sgn * length * cosHeading;
                
                tmp[0] = dx + startCoord[0];
                tmp[1] = dy + startCoord[1];
                tmp[2] = startCoord[2];
                wps.push_back(tmp);
                
                dx += 0.5 * lh;
                
                tmp[0] = dx + startCoord[0];
                wps.push_back(tmp);
                
                        
            }
            
            // set final waypoint precisely
            tmp[0] = length * sinHeading;
            tmp[1] = length * cosHeading;
            tmp[2] = startCoord[2];
            wps.push_back(tmp);
            break;
            
        case CIRCLE:
            // TODO: Create circle type mission
            break;
            
        case SQUARE:
            
            break;
            
        case FIG8:
            // seems to work for all quadrants 
            for (ndx = 0; ndx < npoints; ndx++) 
            {
                t = (ndx * 2 * M_PI / npoints) + M_PI/2;
                tmp[2] = startCoord[2];
                
                // fill x, y
                tmp[0] = 2 / (3 - cos(2 * t)) * cos(t);
                tmp[1] = 2 / (3 - cos(2 * t)) * sin(2 * t) / 2;

                // perform rotation
                theta = (90 - heading) * M_PI/180.0;
                util = tmp[0]; // store old value of 
                tmp[0] = length * (cos(theta) * tmp[0] - sin(theta) * tmp[1]) + startCoord[0];
                tmp[1] = length * (sin(theta) * util + cos(theta) * tmp[1]) + startCoord[1];
                
                wps.push_back(tmp);
            }
            break;
            
        case RACETRACK:
            
                        // seems to work for all quadrants 
            for (ndx = 0; ndx < npoints; ndx++) 
            {
                t = (ndx * 2 * M_PI / npoints) + M_PI/2;
                tmp[2] = startCoord[2];
                
                // fill x, y
                tmp[0] = 2 / (3 - cos(2 * t)) * cos(t);
                tmp[1] = 2 / (3 - cos(2 * t)) * sin(t) / 2;

                // perform rotation
                theta = (90 - heading) * M_PI/180.0;
                util = tmp[0]; // store old value of 
                tmp[0] = length * (cos(theta) * tmp[0] - sin(theta) * tmp[1]) + startCoord[0];
                tmp[1] = length * (sin(theta) * util + cos(theta) * tmp[1]) + startCoord[1];
                
                wps.push_back(tmp);
            }
            break;
            
        case RACETRACK_WIDE:
            
                        // seems to work for all quadrants 
            for (ndx = 0; ndx < npoints; ndx++) 
            {
                t = (ndx * 2 * M_PI / npoints) + M_PI/2;
                tmp[2] = startCoord[2];
                
                // fill x, y
                tmp[0] = 2 / (3 - cos(2 * t)) * cos(t);
                tmp[1] = 2 / (3 - cos(2 * t)) * sin(t);

                // perform rotation
                theta = (90 - heading) * M_PI/180.0;
                util = tmp[0]; // store old value of 
                tmp[0] = length * (cos(theta) * tmp[0] - sin(theta) * tmp[1]) + startCoord[0];
                tmp[1] = length * (sin(theta) * util + cos(theta) * tmp[1])+ startCoord[1];
                
                wps.push_back(tmp);
            }
            break;
            
        default:
            std::cerr << "Invalid pattern type" << std::endl;
            exit(1);
            // TODO: Throw GCS a warning and stop mission
            
    }
}

void waypoints::plotWp()
{
    // TODO: Make this generate plot using gnuplot
    unsigned int ndx;
    for (ndx = 0; ndx < wps.size(); ndx++ ) 
    {
        std::cerr << wps[ndx][0] << "," << wps[ndx][1] << "," << wps[ndx][2] << ";" << std::endl;
    }
}

waypoints::~waypoints() {
}

