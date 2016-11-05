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

#include "waypoints.hpp"
#include "processInterface.hpp"

// TODO: Optimized code by turning doubles to long ints

waypoints::waypoints(configContainer *configs) {
    FOV_V = configs->cam_FOV_v / 57.3;
    FOV_H = configs->cam_FOV_h / 57.3;
    npoints = configs->npoints;
}

void waypoints::calcFOV() {
    // Assuming theta = 0, camera pointed straight down
    float alpha = FOV_V / 2.0; // rads
    float beta = (180 / 57.3 - FOV_H) / 2.0;
    float lc = msnHeight;
    float le = msnHeight / cos(-alpha);
    lv = 2 * sqrt(lc * lc + le * le - 2 * le * lc * cos(alpha));
    lh = (lc / (sin(beta) * sin(beta))) * sin(FOV_H);

}

void waypoints::setWps(coordLocalNED startCoord, int heading, int length, int pattern, float fieldHeading) {
    // fieldHeading is in degrees
    
    msnHeight = abs(startCoord[2]); // assume local NED z = 0 is ground
    calcFOV();
    currentWp = 0; // restart from Wp 0 when new Search Chunk is sent
    wps.clear();
    
    // TODO: Fix these hard coded conversions
    fieldHeading -= 90;
    heading -= fieldHeading;
    
    // Rotation matrix about Z axis 
    Matrix3d DCMz, DCMz_t;
    fieldHeading *= M_PI/180.0; // convert from radians to degrees
    
    DCMz << cos(fieldHeading), sin(fieldHeading), 0, -sin(fieldHeading), cos(fieldHeading), 0, 0, 0, 1;
    DCMz_t = DCMz.transpose();
    std::cout << DCMz << std::endl;
    std::cout << DCMz_t << std::endl;
    
    
    // RECTANGLE Variables
    float dx(0.0), dy(0.0), sgn(0.0);
    int dx_sign(1);
    float sinHeading = sind(heading);
    float cosHeading = cosd(heading);
    coordLocalNED tmp;

    //FIG8 Variables
    unsigned int ndx(0);
    float t(0);
    float theta(0);
    float util(0);

    switch (pattern) {
        case RECTANGLE:
            startCoord = DCMz * startCoord;
            dx_sign = (sinHeading >= 0) ? 1 : -1;

            // Throw this into function so I can create variables inside
            wps.push_back(DCMz * startCoord);

            while (abs(dx) < abs(length * sinHeading)) {
                sgn = 1 - sgn;
                dy = sgn * length * cosHeading;

                tmp[0] = dx + startCoord[0];
                tmp[1] = dy + startCoord[1];
                tmp[2] = startCoord[2];
                std::cout << tmp << std::endl;
                wps.push_back(DCMz * tmp);

                dx += dx_sign * 0.5 * lh;

                tmp[0] = dx + startCoord[0];
                wps.push_back(DCMz * tmp);


            }

            // set final waypoint precisely
            tmp[0] = length * sinHeading + startCoord[0];
            tmp[1] = length * cosHeading + startCoord[1];
            tmp[2] = startCoord[2];
            wps.push_back(DCMz * tmp);
            break;

        case CIRCLE:
            // TODO: Create circle type mission
            break;

        case SQUARE:

            break;

        case FIG8:

            tmp[2] = startCoord[2];
            // seems to work for all quadrants 
            for (ndx = 0; ndx < npoints; ndx++) {
                t = (ndx * 2 * M_PI / npoints) + M_PI / 2;


                // fill x, y
                tmp[0] = 2 / (3 - cos(2 * t)) * cos(t);
                tmp[1] = 2 / (3 - cos(2 * t)) * sin(2 * t) / 2;

                // perform rotation
                theta = (90 - heading) * M_PI / 180.0;
                util = tmp[0]; // store old value of 
                tmp[0] = length * (cos(theta) * tmp[0] - sin(theta) * tmp[1]) + startCoord[0];
                tmp[1] = length * (sin(theta) * util + cos(theta) * tmp[1]) + startCoord[1];

                wps.push_back(tmp);
            }
            break;

        case RACETRACK:

            tmp[2] = startCoord[2];
            // seems to work for all quadrants 
            for (ndx = 0; ndx < npoints; ndx++) {
                t = (ndx * 2 * M_PI / npoints) + M_PI / 2;


                // fill x, y
                tmp[0] = 2 / (3 - cos(2 * t)) * cos(t);
                tmp[1] = 2 / (3 - cos(2 * t)) * sin(t) / 2;

                // perform rotation
                theta = (90 - heading) * M_PI / 180.0;
                util = tmp[0]; // store old value of 
                tmp[0] = length * (cos(theta) * tmp[0] - sin(theta) * tmp[1]) + startCoord[0];
                tmp[1] = length * (sin(theta) * util + cos(theta) * tmp[1]) + startCoord[1];

                wps.push_back(tmp);
            }
            break;

        case RACETRACK_WIDE:

            tmp[2] = startCoord[2];
            // seems to work for all quadrants 
            for (ndx = 0; ndx < npoints; ndx++) {
                t = (ndx * 2 * M_PI / npoints) + M_PI / 2;

                // fill x, y
                tmp[0] = 2 / (3 - cos(2 * t)) * cos(t);
                tmp[1] = 2 / (3 - cos(2 * t)) * sin(t);

                // perform rotation
                theta = (90 - heading) * M_PI / 180.0;
                util = tmp[0]; // store old value of 
                tmp[0] = length * (cos(theta) * tmp[0] - sin(theta) * tmp[1]) + startCoord[0];
                tmp[1] = length * (sin(theta) * util + cos(theta) * tmp[1]) + startCoord[1];

                wps.push_back(tmp);
            }
            break;

        case CAM_ALTITUDE_TEST:

            // Camera Testing Mode where the UAV increases its Altitude at constant intervals

            tmp[0] = startCoord[0];
            tmp[1] = startCoord[1];
            tmp[2] = startCoord[2]; // Initial height is starting Altitude
            wps.push_back(tmp);

            for (ndx = 1; ndx < npoints; ndx++) {
                tmp[2] -= length / npoints;
                wps.push_back(tmp);
            }

            break;

        default:
            std::cerr << "Invalid pattern type" << std::endl;
            exit(1);
            // TODO: Throw GCS a warning and stop mission

    }
}

void waypoints::plotWp() {
    // TODO: Make this generate plot using gnuplot
    unsigned int ndx;
    std::cerr << "wp = [ " << std::endl;
    for (ndx = 0; ndx < wps.size(); ndx++) {
        std::cerr << wps[ndx][0] << "," << wps[ndx][1] << "," << wps[ndx][2] << ";" << std::endl;
    }
    std::cerr << "]" << std::endl;
    std::cerr << "figure; plot(wp(:,1), wp(:,2),'o--'); grid on; axis equal;" << std::endl;
}

coordLocalNED waypoints::LLAtoLocalNED(configContainer& configs, coordLLA &LLA) {

    if (!configs.originSet) {
        std::cerr << "LocalNED frame origin unknown. Call findOriginLocalNED() method with initial position LocalNED and LLA to set origin" << std::endl;
    }
    coordECEF ECEF = LLAtoECEF(LLA);
    std::cout << configs.Rne << std::endl;
    std::cout << configs.originECEF << std::endl;
    std::cout << configs.Rne * (ECEF - configs.originECEF) << std::endl;

    return configs.Rne * (ECEF - configs.originECEF);

}

coordLLA waypoints::LocalNEDtoLLA(configContainer& configs, coordLocalNED &LNED) {
    if (!configs.originSet) {
        std::cerr << "LocalNED frame origin unknown. Call findOriginLocalNED() method with initial position LocalNED and LLA to set origin" << std::endl;
    }
    coordECEF ECEF = configs.Rne_transpose * LNED + configs.originECEF;
    return ECEFtoLLA(ECEF);
}

coordLocalNED waypoints::ECEFtoLocalNED(configContainer& configs, coordECEF &ECEF) {
    if (!configs.originSet) {
        std::cerr << "LocalNED frame origin unknown. Call findOriginLocalNED() method with initial position LocalNED and LLA to set origin" << std::endl;
    }
    return configs.Rne * (ECEF - configs.originECEF);

}

coordECEF waypoints::LocalNEDtoECEF(configContainer& configs, coordLocalNED &LNED) {
    if (!configs.originSet) {
        std::cerr << "LocalNED frame origin unknown. Call findOriginLocalNED() method with initial position LocalNED and LLA to set origin" << std::endl;
    }
    return configs.Rne_transpose * LNED + configs.originECEF;

}

coordLLA waypoints::ECEFtoLLA(coordECEF &ECEF) {
    // function to convert a single coordECEF type coordinate into a coordLLA type coordinate
    // requires coordECEF in meters
    // will return coordLLA in radians
    // TODO: Allow handling matricies of coordinates
    int counter(0);

    double p;
    double N;
    double tolerance = 0.0000000001;
    double X = ECEF(0);
    double Y = ECEF(1);
    double Z = ECEF(2);
    double lam, phi, h(0);
    double phi_1(-1), h_1(-1);


    lam = atan2(Y, X);
    p = sqrt(pow(X, 2) + pow(Y, 2));
    phi = atan2(Z, p * (1 - e2));

    while (abs(phi - phi_1) > tolerance && abs(h - h_1) > tolerance) {

        if (counter > 0) {
            phi = phi_1;
            h = h_1;
        }

        N = a / sqrt(1 - e2 * pow(sin(phi), 2));
        h_1 = p / cos(phi) - N;
        phi_1 = atan2(Z, p * (1 - e2 * (N / (N + h_1))));

        // prevent infinite loop
        if (++counter > 1000) {
            std::cerr << "Warning: ECEFtoLLA conversion failed to converge after 1000 iterations" << std::endl;
            break;
        }

    }

    return coordLLA(phi_1, lam, h_1);
}

coordECEF waypoints::LLAtoECEF(coordLLA &LLA) {
    // function to convert a single coordLLA type coordinate into a coordLLA type coordinate
    // requires coordLLA in radians
    // will return coordECEF in meters

    double phi = LLA(0);
    double lam = LLA(1);
    float h = LLA(2);
    double N;
    double X, Y, Z;

    N = a / sqrt(1 - e2 * pow(sin(phi), 2.0));
    X = (N + h) * cos(phi) * cos(lam);
    Y = (N + h) * cos(phi) * sin(lam);
    Z = (b2 / a2 * N + h) * sin(phi);
    return coordECEF(X, Y, Z);

}

void waypoints::findOriginLocalNED(configContainer& configs, coordLocalNED &LNED, coordLLA &LLA) {
    /* finds the origin of the LocalNED coordinate frame given a LocalNED coordinate and LLA coordinate. Assumes origin
     * is nearby to guarantee accuracy. (accuracy not tested at long ranges > 100m)
     * 
     *  Note: Origin is determined at each power up when GPS lock is obtained. Therefore, the quadcopter should not be 
     * moved far from where it was powered to maintain accuracy of origin calculations
     */

    // TODO: Verify function accuracy at short and long ranges
    float bearing;
    float distance;
    coordLLA originLLA;

    // find heading/bearing to origin
    bearing = atan2(LNED(1), LNED(0)) - M_PI;

    // find distance to origin
    distance = sqrt(pow(LNED(0), 2) + pow(LNED(1), 2) + pow(LNED(2), 2));

    originLLA = coordAt(LLA, bearing, distance);
    originLLA(2) = 0;

    // check if origin is already supposedly set
    if (configs.originSet) {
        // TODO: Handle case if origin is supposedly set already
        std::cerr << "Attempt to set LocalNED origin when origin was already set" << std::endl;
    } else {
        configs.originLLA = originLLA;

        // TODO: See if static method can call another static method within same class 
        configs.originECEF = LLAtoECEF(originLLA);
        configs.originSet = true;
        configs.Rne << -sin(LLA(0)) * cos(LLA(1)), -sin(LLA(0)) * sin(LLA(1)), cos(LLA(0)),
                -sin(LLA(1)), cos(LLA(1)), 0,
                -cos(LLA(0)) * cos(LLA(1)), -cos(LLA(0)) * sin(LLA(1)), -sin(LLA(0));

        configs.Rne_transpose = configs.Rne.transpose();
    }
}

coordLLA waypoints::coordAt(coordLLA &LLA, float bearing, float distance) {
    /* Function to calculate the GPS coordinate at a distance away in bearing direction from an initial GPS coordinate
     * 
     * LLA is in radians and meters
     * distance is in meters
     * bearing is in radians
     * 
     * Reference: Vincenty Formula (Direct Problem) https://en.wikipedia.org/wiki/Vincenty%27s_formulae
     */

    // TODO: Test function
    int counter(0);
    double tolerance(0.0000000000001);
    double lam;
    double U1;
    double sig1;
    double cos2a, sina;
    double u2;
    double A, B;
    double sig_old, sig_new(0);
    double sigM(0);
    double dSig(0);
    double C, L;
    coordLLA LLA2(0, 0, LLA(2));

    U1 = atan((1 - f) * tan(LLA(0)));
    sig1 = atan2(tan(U1), cos(bearing));
    sina = cos(U1) * sin(bearing);
    cos2a = 1 - pow(sina, 2);
    u2 = cos2a * (a2 - b2) / b2;
    A = 1 + u2 / 16384 * (4096 + u2 * (-768 + u2 * (320 - 175 * u2)));
    B = u2 / 1024 * (256 + u2 * (-128 + u2 * (74 - 47 * u2)));

    sig_old = distance / (b * A);

    while (fabs(sig_old - sig_new) >= tolerance) {
        std::cout << fabs(sig_old - sig_new) << std::endl;
        if (counter > 0) sig_old = sig_new;

        sigM = 2 * sig1 + sig_old;
        dSig = B * sin(sig_old) * (cos(sigM) + 0.25 * B * (cos(sig_old) * (-1 + 2 * pow(cos(sigM), 2.0)) - 1 / 6 * B * cos(sigM) * (-3 + 4 * pow(sin(sigM), 2.0) * (-3 + 4 * pow(cos(sigM), 2.0)))));

        sig_new = distance / (b * A) + dSig;
        if (++counter > 1000) {
            std::cerr << "Function coordAt did not converge after 1000 iterations" << std::endl;
            break;
        }
    }

    // sig_new is much smaller here
    LLA2(0) = atan2(sin(U1) * cos(sig_new) + cos(U1) * sin(sig_new) * cos(bearing), (1 - f) * sqrt(pow(sina, 2.0) +
            pow((sin(U1) * sin(sig_new) - cos(U1) * cos(sig_new) * cos(bearing)), 2.0)));
    std::cout << pow((sin(U1) * sin(sig_new) - cos(U1) * cos(sig_new) * cos(bearing)), 2) << "," << fabs(sig_old - sig_new) << std::endl;
    lam = atan2(sin(sig_new) * sin(bearing), cos(U1) * cos(sig_new) - sin(U1) * sin(sig_new) * cos(bearing));
    C = f / 16 * cos2a * (4 + f * (4 - 3 * cos2a));
    L = lam - (1 - C) * f * sina * (sig_new + C * sin(sig_new) * (cos(sigM) + C * cos(sig_new) * (-1 + 2 * pow(cos(sigM), 2))));
    LLA2(1) = L + LLA(1);

    return LLA2;
}

waypoints::~waypoints() {
}

