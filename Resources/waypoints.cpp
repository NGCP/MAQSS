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
  msnHeight = configs->alt; // assume local NED z = 0 is ground
  CalcFOV();
}

void waypoints::CalcFOV() {
  // Assuming theta = 0, camera pointed straight down
  float alpha = FOV_V / 2.0; // rads
  float beta = (180 / 57.3 - FOV_H) / 2.0;
  float lc = msnHeight;
  float le = msnHeight / cos(-alpha);
  lv = 2 * sqrt(lc * lc + le * le - 2 * le * lc * cos(alpha));
  lh = (lc / (sin(beta) * sin(beta))) * sin(FOV_H);
}

void waypoints::SetWps(coordLocalNED start_coord, int heading, int length, float field_heading, unsigned int pattern) {
  /* Function to set the searchChunk waypoints given a startCoord, heading, length, and fieldHeading. All setpoints will be in vehicle Local NED frame
   * NOTE: Only pattern type currently supported is RECTANGLE. All other patterns will throw warnings and yield incorrect setpoints
   * 
   * Inputs:
   *      startCoord    : searchChunk starting coordinate in vehicle LocalNED frame. Use LLAtoLocalNED() to convert a GPS coordinate to the vehicle 
   *                              LocalNED frame. The findOriginLocalNED() method must be called AT LEAST ONCE before LLAtoLocalNED() can be used. All local
   *                              NED coordinates are in meters. Note that the Z direction is positive down
   *      heading        :  searchChunk true heading/bearing in degrees. Points the opposite end of the searchChunk from the startCoord in earth's true
   *                              heading frame. Units are degrees
   *      length           : searchChunk distance in meters. Specifies the distance in "heading" direction to the opposite end of the searchChunk
   *      fieldHeading : true heading/bearing of the field being searched in degrees. This parameter serves to rotate the Local NED coordinate frame 
   *                              prior to the waypoints being generated
   *      pattern         : specifies the pattern type to be generated according to the pattern enum in waypoints.hpp. 
   *                              NOTE: Only RECTANGLE Pattern is currently supported
   * 
   */

  int dy_sign(1);
  float zRot, heading_p;
  float sin_heading, cos_heading;
  float dx(0.0), dy(0.0), sgn(0.0);
  float y_total(0.0), x_total(0.0);
  Matrix3d DCMz;
  Vector3d tmp;

  //FIG8 Variables
  unsigned int ndx(0);
  float t(0);
  float theta(0);
  float util(0);

  //  msnHeight = fabs(start_coord[2]); // assume local NED z = 0 is ground
  //  CalcFOV();
  current_wp = 0; // restart from Wp 0 when new Search Chunk is sent
  wps.clear();

  // find clockwise z rotation to align X axis with fieldHeading
  zRot = field_heading - 90;

  // find heading in rotated coordinate frame;
  heading_p = heading - zRot; // heading' [deg]

  // convert values to radians
  zRot *= M_PI / 180.0;
  heading_p *= M_PI / 180.0;

  // Rotation matrix about Z axis 
  DCMz << cos(zRot), sin(zRot), 0, -sin(zRot), cos(zRot), 0, 0, 0, 1;

  // Calculate and store constants (trig ratios and total distance)
  sin_heading = sin(heading_p);
  cos_heading = cos(heading_p);
  y_total = length * sin_heading;
  x_total = length * cos_heading;

  switch (pattern) {
    case RECTANGLE:
      dy_sign = (y_total >= 0) ? 1 : -1;
      wps.push_back(start_coord); // add initial coordinate 

      // fill setpoints in rotated coordinate frame
      while (fabs(dy) < fabs(y_total)) {
        sgn = 1 - sgn;
        dx = sgn * x_total;

        tmp[0] = dx + start_coord[0];
        tmp[1] = dy + start_coord[1];
        tmp[2] = start_coord[2];
        wps.push_back(tmp);

        dy += dy_sign * 0.5 * lh;

        if (fabs(dy) > fabs(y_total)) break; // if dx would shoot past final setpoint, break                
        tmp[1] = dy + start_coord[1];
        wps.push_back(tmp);
      }

      // set final setpoint precisely
      tmp[0] = x_total + start_coord[0];
      tmp[1] = y_total + start_coord[1];
      tmp[2] = start_coord[2];
      wps.push_back(tmp);

      // rotate back to Local NED coordinate frame
      for (ndx = 0; ndx < wps.size(); ndx++) {
        tmp = wps[ndx]; // wps is a vector of coordLocalNED types (Vector3d))

        // translate to align startCoord with origin
        tmp = tmp - start_coord;

        // rotate to Local NED frame
        tmp = DCMz.transpose() * tmp;

        // translate back to startCoord
        tmp = tmp + start_coord;

        wps[ndx] = tmp;
      }
      break;

      // TODO: Reimplement support for other pattern types
    case CIRCLE:
      // TODO: Create circle type mission
      std::cerr << "CIRCLE Pattern type not implemented" << std::endl;
      break;

    case SQUARE:
      std::cerr << "SQUARE Pattern type not implemented" << std::endl;
      break;

    case FIG8:
      std::cerr << "FIG8 Pattern type currently not supported. Generated set points will be incorrect" << std::endl;

      tmp[2] = start_coord[2];
      // seems to work for all quadrants 
      for (ndx = 0; ndx < npoints; ndx++) {
        t = (ndx * 2 * M_PI / npoints) + M_PI / 2;


        // fill x, y
        tmp[0] = 2 / (3 - cos(2 * t)) * cos(t);
        tmp[1] = 2 / (3 - cos(2 * t)) * sin(2 * t) / 2;

        // perform rotation
        theta = (90 - heading) * M_PI / 180.0;
        util = tmp[0]; // store old value of 
        tmp[0] = length * (cos(theta) * tmp[0] - sin(theta) * tmp[1]) + start_coord[0];
        tmp[1] = length * (sin(theta) * util + cos(theta) * tmp[1]) + start_coord[1];

        wps.push_back(tmp);
      }
      break;

    case RACETRACK:
      std::cerr << "RACETRACK Pattern type currently not supported. Generated set points will be incorrect" << std::endl;

      tmp[2] = start_coord[2];
      // seems to work for all quadrants 
      for (ndx = 0; ndx < npoints; ndx++) {
        t = (ndx * 2 * M_PI / npoints) + M_PI / 2;


        // fill x, y
        tmp[0] = 2 / (3 - cos(2 * t)) * cos(t);
        tmp[1] = 2 / (3 - cos(2 * t)) * sin(t) / 2;

        // perform rotation
        theta = (90 - heading) * M_PI / 180.0;
        util = tmp[0]; // store old value of 
        tmp[0] = length * (cos(theta) * tmp[0] - sin(theta) * tmp[1]) + start_coord[0];
        tmp[1] = length * (sin(theta) * util + cos(theta) * tmp[1]) + start_coord[1];

        wps.push_back(tmp);
      }
      break;

    case RACETRACK_WIDE:
      std::cerr << "RACETRACK_WIDE Pattern type currently not supported. Generated set points will be incorrect" << std::endl;

      tmp[2] = start_coord[2];
      // seems to work for all quadrants 
      for (ndx = 0; ndx < npoints; ndx++) {
        t = (ndx * 2 * M_PI / npoints) + M_PI / 2;

        // fill x, y
        tmp[0] = 2 / (3 - cos(2 * t)) * cos(t);
        tmp[1] = 2 / (3 - cos(2 * t)) * sin(t);

        // perform rotation
        theta = (90 - heading) * M_PI / 180.0;
        util = tmp[0]; // store old value of 
        tmp[0] = length * (cos(theta) * tmp[0] - sin(theta) * tmp[1]) + start_coord[0];
        tmp[1] = length * (sin(theta) * util + cos(theta) * tmp[1]) + start_coord[1];

        wps.push_back(tmp);
      }
      break;

    case CAM_ALTITUDE_TEST:

      // Camera Testing Mode where the UAV increases its Altitude at constant intervals

      tmp[0] = start_coord[0];
      tmp[1] = start_coord[1];
      tmp[2] = start_coord[2]; // Initial height is starting Altitude
      wps.push_back(tmp);

      for (ndx = 1; ndx < npoints; ndx++) {
        tmp[2] -= length / npoints;
        wps.push_back(tmp);
      }
      break;

    default:
      std::cerr << "Invalid pattern type" << std::endl;
      return;
      // TODO: Throw GCS a warning and stop mission
  }
}

void waypoints::SetPOI(coordLocalNED coord) {
  if (!POI.size()) current_wp = 0; // if first POI, reset current_wp
  POI.push_back(coord);
}

void waypoints::PlotWp(configContainer& configs, CoordFrame output_coord_frame, PlotOutput plot_output) {
  /* Function to plot the current waypoints in a specified coordinate frame and format
   * 
   * Inputs
   *      configs - configContainer class which stores coordinates for the LocalNED origin. 
   *                      NOTE: Origin must be located with FindOriginLocalNED() prior to calling this function
   *      output_coord_frame  - desired output coordinate frame. Specified according to the CoordFrame enum
   *                      NOTE: BODY_NED frame is not supported
   *                      NOTE: LLA coordinate frame will be given in degrees. ECEF and LOCAL_NED is in meters
   *      plot_output - desired output format. Specified according to the PlotOutput enum
   *                      NOTE: Only MATLAB_CODE is currently supported.
   * 
   * Output
   *      Will print the output to screen in the requested plot_output format and output_coord_frame frame.
   */
  // TODO: Make this generate plot using gnuplot
  unsigned int ndx;
  Vector3d tmp_coord;
  //  std::vector<std::string> output_strings; // store wps as std::string to be converted to other formats

  // TODO: Make switch case store function pointer to prevent repeating code
  std::cerr << std::setprecision(12);
  switch (output_coord_frame) {
    case CoordFrame::ECEF:
      std::cerr << "wp_ECEF = [ " << std::endl;
      for (ndx = 0; ndx < wps.size(); ndx++) {
        tmp_coord = LocalNEDtoECEF(configs, wps[ndx]);
        std::cerr << tmp_coord[0] << "," << tmp_coord[1] << "," << tmp_coord[2] << ";" << std::endl;
      }
      std::cerr << "]" << std::endl;
      std::cerr << "figure; plot(wp_ECEF(:,2), wp_ECEF(:,1),'o--'); grid on; axis equal;" << std::endl;
      break;

    case CoordFrame::LLA:
      std::cerr << "wp_LLA = [ " << std::endl;
      for (ndx = 0; ndx < wps.size(); ndx++) {
        tmp_coord = LocalNEDtoLLA(configs, wps[ndx], AngleType::DEGREES);
        std::cerr << tmp_coord[0] << "," << tmp_coord[1] << "," << tmp_coord[2] << ";" << std::endl;
      }
      std::cerr << "]" << std::endl;
      std::cerr << "figure; plot(wp_LLA(:,2), wp_LLA(:,1),'o--'); grid on; axis equal;" << std::endl;
      break;

    case CoordFrame::BODY_NED:
      // Output Message if BODY_NED selected and switch-case fall through to LOCAL_NED frame
      std::cerr << "Plotting waypoints in Body NED frame NOT supported. Defaulting to Local NED..." << std::endl;

    case CoordFrame::LOCAL_NED:
      std::cerr << "wp_LOCAL_NED = [ " << std::endl;
      for (ndx = 0; ndx < wps.size(); ndx++) {
        std::cerr << wps[ndx][0] << "," << wps[ndx][1] << "," << wps[ndx][2] << ";" << std::endl;
      }
      std::cerr << "]" << std::endl;
      std::cerr << "figure; plot(wp_LOCAL_NED(:,2), wp_LOCAL_NED(:,1),'o--'); grid on; axis equal;" << std::endl;
      break;
  }
  std::cerr << "Lh: " << lh << ", Lv: " << lv << std::endl;
}

coordLocalNED waypoints::LLAtoLocalNED(configContainer& configs, coordLLA &LLA, AngleType input_angle) {
  /* Function to convert a coord in the LLA frame to a coord in the LocalNED frame given the origin in configs
   * 
   * Inputs
   *      configs - configContainer class which stores coordinates for the LocalNED origin. 
   *                      NOTE: Origin must be located with FindOriginLocalNED() prior to calling this function
   *      LLA - coordinate in the LLA frame. The angle is specified to be either radians or degrees
   *      input_angle - specifies if LLA is given in degrees or radians. A LLA input in degrees will be converted to radian
   *                            for computation. Specified according to the AngleType enum
   * 
   * Output
   *      Returns a coordinate in the LOCAL_NED coordinate frame. This coordinate will be in meters
   */
  if (!configs.originSet) {
    throw "LocalNED origin not set. Call FindOriginLocalNED() with initial position LocalNED and LLA to set origin";
  }
  coordECEF ECEF = LLAtoECEF(LLA);
  if (input_angle == AngleType::DEGREES) {
    LLA[0] = LLA[0] * M_PI / 180.0; // convert to radians in necessary
    LLA[1] = LLA[1] * M_PI / 180.0;
  }
  //  std::cout << configs.Rne << std::endl;
  //  std::cout << configs.originECEF << std::endl;
  //  std::cout << configs.Rne * (ECEF - configs.originECEF) << std::endl;

  return configs.Rne * (ECEF - configs.originECEF);
}

coordLLA waypoints::LocalNEDtoLLA(configContainer& configs, coordLocalNED &LNED, AngleType output_angle) {
  /* Function to convert a coord in the LocalNED frame to a coord in the LLA frame given the origin in configs
   * 
   * Inputs
   *      configs - configContainer class which stores coordinates for the LocalNED origin. 
   *                      NOTE: Origin must be located with FindOriginLocalNED() prior to calling this function
   *      LNED - coordinate in the LocalNED frame.
   *      output_angle - specifies if LLA is returned in degrees or radians
   * 
   * Output
   *      Returns a coordinate in the LLA coordinate frame. The angle is specified to be either radians or degrees
   */
  if (!configs.originSet) {
    throw "LocalNED origin not set. Call FindOriginLocalNED() with initial position LocalNED and LLA to set origin";
  }
  coordECEF ECEF = configs.Rne_transpose * LNED + configs.originECEF;
  return ECEFtoLLA(ECEF, output_angle);
}

coordLocalNED waypoints::ECEFtoLocalNED(configContainer& configs, coordECEF &ECEF) {
  if (!configs.originSet) {
    throw "LocalNED origin not set. Call FindOriginLocalNED() with initial position LocalNED and LLA to set origin";
  }
  return configs.Rne * (ECEF - configs.originECEF);

}

coordECEF waypoints::LocalNEDtoECEF(configContainer& configs, coordLocalNED &LNED) {
  if (!configs.originSet) {
    throw "LocalNED origin not set. Call FindOriginLocalNED() with initial position LocalNED and LLA to set origin";
  }
  return configs.Rne_transpose * LNED + configs.originECEF;

}

coordLLA waypoints::ECEFtoLLA(coordECEF &ECEF, AngleType output_angle) {
  /* Function to convert a coord in the ECEF frame to a coord in the LLA frame
   * 
   * Inputs
   *      ECEF - coordinate in the ECEF frame. ECEF coordinates should always be in meters
   *      output_angle - specifies if LLA is returned in degrees or radians
   * 
   * Output
   *      Returns a coordinate in the LLA coordinate frame. The angle is specified to be either radians or degrees
   */
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

  while (fabs(phi - phi_1) > tolerance && fabs(h - h_1) > tolerance) {

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
  if (output_angle == AngleType::DEGREES) {
    phi_1 *= 180.0 / M_PI;
    lam *= 180.0 / M_PI;
  }
  return coordLLA(phi_1, lam, h_1);
}

coordECEF waypoints::LLAtoECEF(coordLLA &LLA, AngleType input_angle) {
  /* Function to convert a coord in the LLA frame to a coord in the ECEF frame
   * 
   * Inputs
   *      LLA - coordinate in the LLA frame. The angle is specified to be either radians or degrees
   *      input_angle - specifies if LLA is given in degrees or radians. 
   * 
   * Output
   *      Returns a coordinate in the ECEF coordinate frame. ECEF coordinates should always be in meters
   */
  double phi = LLA(0);
  double lam = LLA(1);
  float h = LLA(2);
  double N;
  double X, Y, Z;

  if (input_angle == AngleType::DEGREES) {
    phi *= M_PI / 180.0;
    lam *= M_PI / 180.0;
  }

  N = a / sqrt(1 - e2 * pow(sin(phi), 2.0));
  X = (N + h) * cos(phi) * cos(lam);
  Y = (N + h) * cos(phi) * sin(lam);
  Z = (b2 / a2 * N + h) * sin(phi);
  return coordECEF(X, Y, Z);
}

void waypoints::FindOriginLocalNED(configContainer& configs, coordLocalNED &LNED, coordLLA &LLA) {
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
    //    std::cout << fabs(sig_old - sig_new) << std::endl;
    if (counter > 0) sig_old = sig_new;

    sigM = 2 * sig1 + sig_old;
    dSig = B * sin(sig_old) * (cos(sigM) + 0.25 * B * (cos(sig_old) * (-1 + 2 * pow(cos(sigM), 2.0)) - 1 / 6 * B * cos(sigM) * (-3 + 4 * pow(sin(sigM), 2.0) * (-3 + 4 * pow(cos(sigM), 2.0)))));

    sig_new = distance / (b * A) + dSig;
    if (++counter > 1000) {
      std::cerr << "Function coordAt did not converge after 1000 iterations" << std::endl;
      break;
    }
  }

  LLA2(0) = atan2(sin(U1) * cos(sig_new) + cos(U1) * sin(sig_new) * cos(bearing), (1 - f) * sqrt(pow(sina, 2.0) +
          pow((sin(U1) * sin(sig_new) - cos(U1) * cos(sig_new) * cos(bearing)), 2.0)));
  lam = atan2(sin(sig_new) * sin(bearing), cos(U1) * cos(sig_new) - sin(U1) * sin(sig_new) * cos(bearing));
  C = f / 16 * cos2a * (4 + f * (4 - 3 * cos2a));
  L = lam - (1 - C) * f * sina * (sig_new + C * sin(sig_new) * (cos(sigM) + C * cos(sig_new) * (-1 + 2 * pow(cos(sigM), 2))));
  LLA2(1) = L + LLA(1);

  return LLA2;
}

waypoints::~waypoints() {
}

