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
// Example GCS mission: "Q2,P35.30824330314721 -120.66806624303483 -0.5985199129208922,H-11.191466156610467,F139.0,D123.00600959784066"
struct search_chunk {
  std::string msg;
  double lat = 0; // [deg]
  double lon = 0; // [deg]
  float alt = 0; // [m]
  float heading = 0; // [deg]
  float field_heading = 0; // [deg]
  float distance = 0; // [m]
  bool changed_flag = false;

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

void callback_fun(XBEE::Frame *item) {

  // ReceivePacket pointer
  // dynamic cast to type of frame I think it is (ReceivePacket), store in pointer
  XBEE::ReceivePacket *r_packet = dynamic_cast<XBEE::ReceivePacket*> (item);
  std::string str_data;
  std::string tmp_chars;
  std::string position_string;
  std::vector<std::string> position_strings(3);
  std::string heading_string;
  std::string fheading_string;
  std::string dist_string;
  int delim = ' ';

  // check if pointer is NULL
  if (r_packet != NULL) {
    str_data = r_packet->GetData();
  }

  // TODO: Get data from message and store into search_chunk struct
  std::cerr << "Receved Msg: " << str_data << std::endl;
  search_chunk.msg = str_data;
  tmp_chars = strtok((char*) str_data.c_str(), ","); // Read the Q token
  if (tmp_chars[0] != 'Q') return; // if not the expected message type, throw out to prevent crash

  // TODO: Clean this up using the split function
  // TODO: Check that message is directed to correct quad?
  int debug = 0;
  std::cerr << debug++ << std::endl;
  tmp_chars = strtok(NULL, ","); // Read the P token
  position_string = tmp_chars;
  position_string.erase(0, 1);
  position_strings = split(position_string, delim);
  std::cerr << "Position String: " << position_string << ", Delimiter: " << delim << std::endl;
  std::cerr << "0: " << position_strings[0] << ", 1: " << position_strings[1] << " ,2: " << position_strings[2] << std::endl;
  search_chunk.lat = std::stod(position_strings[0]);
  search_chunk.lon = std::stod(position_strings[1]);
  search_chunk.alt = std::stof(position_strings[2]);

  std::cerr << debug++ << std::endl;
  tmp_chars = strtok(NULL, ","); // Read the H token
  heading_string = tmp_chars;
  heading_string.erase(0, 1);
  search_chunk.heading = std::stof(heading_string);
  
  std::cerr << debug++ << std::endl;
  tmp_chars = strtok(NULL, ","); // Read the H token
  fheading_string = tmp_chars;
  fheading_string.erase(0, 1);
  search_chunk.field_heading = std::stof(fheading_string);

  std::cerr << debug++ << std::endl;
  tmp_chars = strtok(NULL, ","); // Read the D token
  dist_string = tmp_chars;
  dist_string.erase(0, 1);
  search_chunk.distance = std::stof(dist_string);

  //  std::cerr << debug++ << std::endl;
  //  tmp_chars = strtok(NULL, ","); // Read any extra tokens
  //
  //  std::cerr << debug++ << std::endl;
  //  while (tmp_chars.c_str() != NULL) {
  //    std::cerr << "Read Extra messages: " << tmp_chars << std::endl;
  //    tmp_chars = strtok(NULL, ",");
  //  }

  std::cerr << debug++ << std::endl;
  // set flag to indicate new mission
  search_chunk.changed_flag = true;
  std::cerr << debug++ << std::endl;

}

int mainLoop(processInterface *PNav, configContainer *configs) {

  // TODO: Make a comms(RBP test mode which doesnt try to start the autopilot_interface)  test loop
  using namespace std::chrono;

  // Declare variables
  bool set_point_reached = false;
  bool cv_busy = false;
  bool offboard = false;
  unsigned int ndx(0);
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
  xbee_interface.ReadHandler = std::bind(&callback_fun, std::placeholders::_1);
  xbee_interface.AsyncReadFrame();

  // declare variables for mavlink messages
  mavlink_set_position_target_local_ned_t sp;
  mavlink_set_position_target_local_ned_t ip;
  mavlink_local_position_ned_t lpos;
  Mavlink_Messages msgs;
  //    mavlink_position_target_local_ned_t tpos;

  // start timer for logging
  steady_clock::time_point t0, t1;
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
  waypoints::FindOriginLocalNED(*configs, LNED_0, LLA_0);

  //TODO: Write function/method to receive mission data (GPS Start, heading, distance)

  // Fly InputFile pattern if specified
  if (configs->pattern != 999) pattern = configs->pattern;
  else {
  } // let GCS specify

  // Fly InputFile mission if specified
  if (configs->head != 999 && configs->dist != 0) searchChunk.SetWps(startCoord, configs->head, configs->dist, configs->field_heading, pattern);
  //    else searchChunk.setWps(startCoord, 100, 50, 139, RECTANGLE);

  std::cerr << "New Search Chunk Set with Parameters: heading = " << configs->head << ", distance: " << configs->dist << std::endl;
  std::cerr << std::endl;

  if (pattern == CAM_ALTITUDE_TEST) configs->setpoint_tolerance = 0.5; // reduce setpoint tolerance for camera altitude test to make sure AV stops at each interval

  std::cerr << "Current IP: [" << ip.x << ", " << ip.y << "," << ip.z << "]" << std::endl;
  std::cerr << "Begin Streaming setpoints:" << std::endl;
  std::cerr << "sp = [" << std::endl;
  //  for (ndx = 0; ndx < searchChunk.wps.size(); ndx++) {
  //    t0 = steady_clock::now();
  //    set_position(searchChunk.wps[ndx][0], searchChunk.wps[ndx][1], searchChunk.wps[ndx][2], sp);
  //    autopilot_interface.update_setpoint(sp);
  //    set_point_reached = false;
  //
  //    //        PNav->writePipe(configs->fd_PNav_to_CV, cv_msg);
  //
  //    std::cerr << ndx + 1 << ", " << searchChunk.wps[ndx][0] << ", " << searchChunk.wps[ndx][1] << ", " << searchChunk.wps[ndx][2] << "; " << std::endl;
  //    while (!set_point_reached) {
  //
  //      //            tpos = autopilot_interface.current_messages.position_target_local_ned;
  //      //            std::cerr << tpos.x << ", " << tpos.y << ", " << tpos.z << std::endl;
  //      //            sleep(1);
  //      //            if (abs(tpos.x - sp.x) < configs->setpoint_tolerance  && abs(tpos.y - sp.y) < configs->setpoint_tolerance  && abs(tpos.z - sp.z) < configs->setpoint_tolerance ) {
  //      //                set_point_reached = true;
  //      //                break;
  //      //            }
  //
  //      lpos = autopilot_interface.current_messages.local_position_ned;
  //      if (abs(lpos.x - sp.x) < configs->setpoint_tolerance && abs(lpos.y - sp.y) < configs->setpoint_tolerance && abs(lpos.z - sp.z) < configs->setpoint_tolerance) {
  //        set_point_reached = true;
  //        break;
  //      }
  //
  //      /* 
  //       * autopilot_interface.current_messages.heartbeat.custom_mode will return the following values to correspond to flight modes
  //       * NOTE: Needs more testing to verify
  //       * offboard = 393216 ->     0x00060000 = 00000000 00000110 00000000 00000000
  //       * Manual  =   65536  ->     0x00010000 = 00000000 00000001 00000000 00000000
  //       * Position C = 196608 ->   0x00030000 = 00000000 00000011 00000000 00000000
  //       * Land = 100925440 ->     0x06040000 = 00000110 00000100 00000000 00000000
  //       * 
  //       */
  //
  //      t1 = steady_clock::now();
  //      if (configs->log && (((duration_cast<milliseconds>(t1 - t0).count()) > (1 / configs->log_freq) * 1000))) {
  //
  //        flt_log.log(&autopilot_interface.current_messages);
  //        t0 = steady_clock::now();
  //      }
  //      ndx = ((ndx == configs->npoints - 1) ? 0 : ndx);
  //
  //      // if offboard is toggled, request a frame capture from CV
  //      offboard = ((0x00060000 & autopilot_interface.current_messages.heartbeat.custom_mode) == 393216);
  //      if (offboard && !cv_busy) {
  //        PNav->writePipe(configs->fd_PNav_to_CV, cv_msg);
  //        cv_busy = true; // flag cv_process as busy so multiple "Starts" dont get sent
  //      }
  //
  //      // if msg is read and msg says "Done", reset cv_busy flag (function calls should resolve Left -> right)
  //      if (read(configs->fd_CV_to_PNav, tmp, BUF_LEN) && !strcmp(tmp, "Done")) cv_busy = false;
  //
  //      // clear read buffer at every loop;
  //      tmp[0] = '\0';
  //    }
  //  }
  //  std::cerr << "];" << std::endl;
  //  std::cerr << "plot(sp(:,2),sp(:,3),'-x')\nhold all\n axis equal\ngrid on\nplot(sp(1,2),sp(1,3),'or')" << std::endl;
  //
  //  cv_msg = "Exit";
  //  PNav->writePipe(configs->fd_PNav_to_CV, cv_msg);

  // switch through (read from CV, check setpoint, check GCS comms, write GPS to GCS if past 1s)
  std::cerr << "Starting comms test loop" << std::endl;
  while (1) {
    // if read(gcsInterface) has information
    std::cout << "New Mission: " << search_chunk.changed_flag << std::endl;
    if (search_chunk.changed_flag) {
      std::cerr << "New Mission Received" << std::endl;
      std::cerr << "Starting [lat, lon, alt]: [" << search_chunk.lat << ", " << search_chunk.lon << ", " << search_chunk.alt << "]" << std::endl;
      std::cerr << "Mission [head, dist]: [" << search_chunk.heading << ", " << search_chunk.distance << "]" << std::endl;
      search_chunk.changed_flag = false;

      // calculate wp
      // TODO: Altitude should be set from input file
      start_coordLLA  << (float)search_chunk.lat, (float)search_chunk.lon, configs->alt;
      start_coordLLA[0] = start_coordLLA[0] * M_PI/180.0;// convert from deg2rad
      start_coordLLA[1] = start_coordLLA[1] * M_PI/180.0;
      startCoord = waypoints::LLAtoLocalNED(*configs, start_coordLLA);
      searchChunk.SetWps(startCoord, search_chunk.heading, search_chunk.distance, search_chunk.field_heading, pattern);

      std::cerr << "New Search Chunk Set with Parameters: heading = " << search_chunk.heading << 
              ", field_heading: " << search_chunk.field_heading <<
              ", distance: " << search_chunk.distance << std::endl;
      searchChunk.PlotWp(*configs);
      searchChunk.PlotWp(*configs, CoordFrame::LLA);
      std::cerr << std::endl;
    }
    usleep(500000);
    // if start command, set flag, (tell CV to start?)

    // if stop command, set flag (tell CV to stop?)

    // if Search Chunk
    // Update Waypoint class
    // Reset current waypoint index to 0 (beginning)

    // if start command set 
    // if time change > .1s, send a GPS location message

    // if read(CV) has message
    // if point of interest found, send GPS location to GCS

    // if current location is within tolerance of target setpoint
    // increment current waypoint index
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
