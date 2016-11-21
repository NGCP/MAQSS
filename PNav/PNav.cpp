// C++ headers
#include <cstdlib>
#include <iostream>
#include <fstream>
#include <cstring>
#include <array>
#include <vector>
#include <cmath>
#include <ctime>
#include <chrono>
#include <functional>

// C headers
#include <fcntl.h>
#include <stdio.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <signal.h>

// Custom headers
#include "fileIO.hpp"
#include "processInterface.hpp"
#include "waypoints.hpp"
#include "autopilot_interface.h"
#include "serial_port.h"
#include "flight_logger.hpp"
#include <SerialXbee.hpp>
#include <ReceivePacket.hpp>

// TODO: Have karthik fix my makefile
// scp -r zhangh94@10.42.0.1:/home/zhangh94/NGCP/MAQSS .

// ------------------------------------------------------------------------------
//   Quit Signal Handler
// ------------------------------------------------------------------------------
// this function is called when you press Ctrl-C

Autopilot_Interface *autopilot_interface_quit;
Serial_Port *serial_port_quit;
processInterface *PNav_quit;
configContainer *configs_quit;

void quit_handler(int sig) {
  printf("\n");
  printf("TERMINATING AT USER REQUEST\n");
  printf("\n");

  // autopilot interface
  try {
    autopilot_interface_quit->handle_quit(sig);
  } catch (int error) {
  }

  // serial port
  try {
    serial_port_quit->handle_quit(sig);
  } catch (int error) {
  }

  // pipes
  PNav_quit->cleanup(configs_quit);

  // end program here
  exit(0);
}

// callback function for xbee_interface
// Example GCS mission: "NEWMSG,MSN,Q2,P35.308 -120.668 -0.5985199129208922, H-11.191,F139.0,D123"

struct search_chunk {
  std::string msg;
  double lat = 0; // [deg]
  double lon = 0; // [deg]
  float alt = 0; // [m]
  float heading = 0; // [deg]
  float field_heading = 0; // [deg]
  float distance = 0; // [m]
  bool changed_flag = false;
  bool start = false;

} search_chunk;

std::vector<std::string> split(const std::string &s, char delim) {
  std::stringstream ss(s);
  std::string item;
  std::vector<std::string> tokens;
  while (getline(ss, item, delim)) {
    tokens.push_back(item);
  }
  return tokens;
}

void CallbackFunction(XBEE::Frame *item) {

  // ReceivePacket pointer
  // dynamic cast to type of frame I think it is (ReceivePacket), store in pointer
  XBEE::ReceivePacket *r_packet = dynamic_cast<XBEE::ReceivePacket*> (item);
  std::vector<std::string> msg_components;
  bool valid_msg = true;
  std::string str_data;
  std::vector<std::string> position_strings(3);
  int delim = ',';

  // check if pointer is NULL
  if (r_packet != NULL) {
    str_data = r_packet->GetData();
  }

  // check char[6] == ',' (comma) (prevent crashes))
  if (str_data[6] != ',') valid_msg = false;
  else {
    // split message
    msg_components = split(str_data, delim);

    // check first string is NEWMSG. If not, break
    if (strcmp(msg_components[0].c_str(), "NEWMSG")) valid_msg = false;

    // if-else if branch to determine message type and handling
    if (!strcmp(msg_components[1].c_str(), "MSN") && valid_msg) {
      // NEWMSG,MSN,Q2,P35.3085519592 -120.668932266 0.00000,H103.29054493,F139.7,D228.812279
      delim = ' ';
      std::cerr << "Handling Mission Message: " << str_data << std::endl;

      // store search_chunk start position
      msg_components[3].erase(0, 1);
      position_strings = split(msg_components[3], delim);
      search_chunk.lat = std::stod(position_strings[0]);
      search_chunk.lon = std::stod(position_strings[1]);
      search_chunk.alt = std::stof(position_strings[2]);

      // store search_chunk mission heading
      msg_components[4].erase(0, 1);
      search_chunk.heading = std::stof(msg_components[4]);

      // store search_chunk field heading
      msg_components[5].erase(0, 1);
      search_chunk.field_heading = std::stof(msg_components[5]);

      // store search_chunk distance
      msg_components[6].erase(0, 1);
      search_chunk.distance = std::stof(msg_components[6]);

      // set flag to indicate new mission parameters received
      search_chunk.changed_flag = true;

    } else if (!strcmp(msg_components[1].c_str(), "START") && valid_msg) {
      // NEWMSG,START
      std::cerr << "Handle Start Message: " << str_data << std::endl;
      search_chunk.start = true;
    } else if (!strcmp(msg_components[1].c_str(), "STOP") && valid_msg) {
      // NEWMSG,STOP
      std::cerr << "Handle Stop Message: " << str_data << std::endl;
      search_chunk.start = false;
    } else if (!strcmp(msg_components[1].c_str(), "POI") && valid_msg) {
      // NEWMSG,POI
      std::cerr << "Handle Point of Interest Message: " << str_data << std::endl;
    } else valid_msg = false;
  }

  // If msg was invalid, output error msg to screen
  if (!valid_msg) {
    std::cerr << "Invalid Msg Read: " << str_data << std::endl;
    std::cerr << "Message ignored!" << std::endl;
  }
}

int mainLoop(processInterface *PNav, configContainer *configs) {

  // TODO: Make a comms(RBP test mode which doesnt try to start the autopilot_interface)  test loop
  using namespace std::chrono;

  // Declare variables
  bool set_point_reached = false;
  bool cv_busy = false;
  bool offboard = false;
  unsigned int ndx(0), wp_ndx(0);
  int pattern(0);
  char tmp[BUF_LEN];
  coordLLA start_coordLLA;
  coordLocalNED startCoord;
  std::string cv_msg = "Start";

  // Setup Autopilot interface
  Serial_Port serial_port(configs->uart_name.c_str(), configs->baudrate);
  Autopilot_Interface autopilot_interface(&serial_port);

  // Setup Xbee serial interface 
  XBEE::SerialXbee xbee_interface;
  xbee_interface.ReadHandler = std::bind(&CallbackFunction, std::placeholders::_1);
  xbee_interface.AsyncReadFrame();

  // declare variables for mavlink messages
  mavlink_set_position_target_local_ned_t sp;
  mavlink_set_position_target_local_ned_t ip;
  mavlink_local_position_ned_t lpos;
  Mavlink_Messages msgs;
  //    mavlink_position_target_local_ned_t tpos;

  // start timer for logging
  steady_clock::time_point t0_log, t0_heartbeat, t1_log, t1_heartbeat;
  flight_logger flt_log;

  // Setup interrupt handlers so all interfaces get closed
  serial_port_quit = &serial_port;
  autopilot_interface_quit = &autopilot_interface;
  PNav_quit = PNav;
  configs_quit = configs;
  signal(SIGINT, quit_handler);

  // Allow camera test mode
  if (configs->cam_Test) {
    std::cerr << "Camera test mode, PNav idling" << std::endl;
    while (true);
  }

  //  Start interface and take initial position
  serial_port.start();
  autopilot_interface.start();
  ip = autopilot_interface.initial_position;

  // TODO: Figure out how to correctly specify height
  startCoord << ip.x, ip.y, -configs->alt; // Assumes start position will be on ground

  // instantiate a waypoints class
  waypoints searchChunk(configs);

  // Locate LNED frame origin
  // TODO: Implement method which checks LNED origin remains constant throughout flight
  msgs = autopilot_interface.current_messages;
  coordLocalNED LNED_0(msgs.local_position_ned.x, msgs.local_position_ned.y, msgs.local_position_ned.z);
  coordLLA LLA_0(msgs.global_position_int.lat * 1E-7 * M_PI / 180.0, msgs.global_position_int.lon * 1E-7 * M_PI / 180.0, msgs.global_position_int.alt * 1E-3);
  
  // TODO: Print origin too
  waypoints::FindOriginLocalNED(*configs, LNED_0, LLA_0);

  //TODO: Write function/method to receive mission data (GPS Start, heading, distance)

  // Fly InputFile pattern if specified
  if (configs->pattern != 999) pattern = configs->pattern;
  else {
  } // let GCS specify

  // Fly InputFile mission if specified
  if (configs->head != 999 && configs->dist != 0) searchChunk.SetWps(startCoord, configs->head, configs->dist, configs->field_heading, pattern);
  //    else searchChunk.setWps(startCoord, 100, 50, 139, RECTANGLE);

  if (pattern == CAM_ALTITUDE_TEST) configs->setpoint_tolerance = 0.5; // reduce setpoint tolerance for camera altitude test to make sure AV stops at each interval

  // store timer for logging and writing heartbeat msgs
   t0_log = steady_clock::now();
   t0_heartbeat = steady_clock::now();
   ndx = 0;
   
  // mainloop
  while (1) {
    // GCS reads are handled by the CallbackFunction
    // check if vehicle is in offboard mode
    offboard = ((0x00060000 & autopilot_interface.current_messages.heartbeat.custom_mode) == 393216);
    
    // set current wp
    ndx = searchChunk.current_wp;
    set_position(searchChunk.wps[ndx][0], searchChunk.wps[ndx][1], searchChunk.wps[ndx][2], sp);
    
    // check current location
    lpos = autopilot_interface.current_messages.local_position_ned;

    usleep(500000);
    // if start command, set flag, (tell CV to start?)

    // if stop command, set flag (tell CV to stop?)

    // if new search_chunk mission msg received, update Waypoint class
    if (search_chunk.changed_flag) {
      std::cerr << "New Mission Received" << std::endl;
      std::cerr << "Starting [lat, lon, alt]: [" << search_chunk.lat << ", " << search_chunk.lon << ", " << search_chunk.alt << "]" << std::endl;
      std::cerr << "Mission [head, dist]: [" << search_chunk.heading << ", " << search_chunk.distance << "]" << std::endl;
      search_chunk.changed_flag = false;

      // calculate wp
      // TODO: Altitude should be set from input file
      start_coordLLA << (float) search_chunk.lat, (float) search_chunk.lon, configs->alt;
      start_coordLLA[0] = start_coordLLA[0] * M_PI / 180.0; // convert from deg2rad
      start_coordLLA[1] = start_coordLLA[1] * M_PI / 180.0;
      startCoord = waypoints::LLAtoLocalNED(*configs, start_coordLLA);
      searchChunk.SetWps(startCoord, search_chunk.heading, search_chunk.distance, search_chunk.field_heading, pattern);

      std::cerr << "New Search Chunk Set with Parameters: heading = " << search_chunk.heading <<
              ", field_heading: " << search_chunk.field_heading <<
              ", distance: " << search_chunk.distance << std::endl;
      searchChunk.PlotWp(*configs);
      searchChunk.PlotWp(*configs, CoordFrame::LLA);
      std::cerr << std::endl;
    }

    // if start command set 
    // if time change > .1s, send a GPS location message

    // if read(CV) has message
    // if point of interest found, send GPS location to GCS

    // if current location is within tolerance of target setpoint
    // increment current waypoint index
    if (abs(lpos.x - sp.x) < configs->setpoint_tolerance && abs(lpos.y - sp.y) < configs->setpoint_tolerance && abs(lpos.z - sp.z) < configs->setpoint_tolerance) {
          set_point_reached = true;
          break;
        }
    
    // if current waypoint index is past end of waypoints vector, tell GCS and loiter
    // update stream to Pixhawk
  }
  std::cerr << "Exiting Comms test loop" << std::endl;

  autopilot_interface.stop();
  serial_port.stop();
  return 0;
}

int main(int argc, char** argv) {
  configContainer configs;
  if (argc > 1) configs = fileIO::getConfig(argc, argv);
  else {
    // TODO: Add a print help function and figure out where to put it
    std::cerr << "Must specify command line arguments" << std::endl;
    exit(1); // TODO: Perform closeout/ clean up function when exiting
  }
  processInterface PNav(&configs, PNAV);
  fileIO::printConfig(&configs);
  mainLoop(&PNav, &configs);

  PNav.cleanup(&configs);

  return 0;
}
