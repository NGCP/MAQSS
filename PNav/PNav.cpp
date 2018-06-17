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
#include <thread>

// C headers
#include <fcntl.h>
#include <stdio.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <signal.h>

#include <time.h>
// Custom headers
#include "fileIO.hpp"
#include "offboard.hpp"
#include "waypoints.hpp"
#include "autopilot_interface.h"
#include "serial_port.h"
#include "flight_logger.hpp"
#include <SerialXbee.hpp>
#include <ReceivePacket.hpp>
#include <TransmitRequest.hpp>
#include "PNav.hpp"

#ifdef EMULATION
#define CALPOLY_LAT 353002209
#define CALPOLY_LON -1206618691
#define CALPOLY_ALT 101602
#endif


// TODO: Have karthik fix my makefile
// scp -r zhangh94@10.42.0.1:/home/zhangh94/NGCP/MAQSS .

bool PNav_shutdown = false;
#ifndef EMULATION
Autopilot_Interface *autopilot_interface_quit;
Serial_Port *serial_port_quit;
#endif

void PNav_call_stop()
{
  // autopilot interface

  PNav_shutdown = true;
  PeeToCee.set_CV_start(false);
  #ifndef EMULATION
  try
  {
    autopilot_interface_quit->handle_quit(SIGINT);
  }
  catch (int error)
  {
  }

  try
  {
    serial_port_quit->handle_quit(SIGINT);
  }
  catch (int error)
  {
  }
  #endif
  std::cerr << "PNav Quitting\n";
  exit(0);
}

struct mission_status
{
  coordLLA target_LLA;
  std::string msg;
  std::string poi_id;
  std::string mission_type;
  double lat = 0;          // [deg]
  double lon = 0;          // [deg]
  float alt = 0;           // [m]
  float heading = 0;       // [deg]
  float field_heading = 0; // [deg]
  float distance = 0;      // [m]
  bool changed_flag = false;
  bool start = false; // commanded start/stop status

} mission_status;

struct vehicle_status
{
  bool start = false; // vehicle mission start/stop status
  bool role_changed = false;
  unsigned int role = 0;
  double lat;
  double lon;
  double alt;
  coordLocalNED LNED;
  std::string status = "Online";
  std::string gcs_update;
} vehicle_status;


std::vector<std::string> split(const std::string &s, char delim)
{
  std::stringstream ss(s);
  std::string item;
  std::vector<std::string> tokens;
  while (getline(ss, item, delim))
  {
    tokens.push_back(item);
  }
  return tokens;
}

// callback function for xbee_interface
// Example GCS mission: "NEWMSG,MSN,Q2,P35.308 -120.668 -0.5985199129208922, H-11.191,F139.0,D123"

void CallbackFunction(XBEE::Frame *item)
{

  // ReceivePacket pointer
  // dynamic cast to type of frame I think it is (ReceivePacket), store in pointer
  XBEE::ReceivePacket *r_packet = dynamic_cast<XBEE::ReceivePacket *>(item);
  std::vector<std::string> msg_components;
  bool valid_msg = true;
  std::string str_data;
  std::vector<std::string> position_strings(3);
  int delim = ',';

  int debug = 0;
  std::cerr << "Entered Callback " << std::endl;
  // check if pointer is NULL
  if (r_packet != NULL)
  {
    str_data = r_packet->GetData();
  }
  std::cerr << debug++ << std::endl;

  // check char[6] == ',' (comma) (prevent crashes))
  if (str_data[6] != ',')
    valid_msg = false;
  else
  {
    // split message
    msg_components = split(str_data, delim);

    // check first string is NEWMSG. If not, break
    if (strcmp(msg_components[0].c_str(), "NEWMSG"))
      valid_msg = false;

    // if-else if branch to determine message type and handling
    if (!strcmp(msg_components[1].c_str(), "MSN") && valid_msg)
    {
      // NEWMSG,MSN,Q2,P35.3085519592:-120.668932266:0,H103.29054493,F139.7,D228.8
      delim = ' ';
      std::cerr << "Handling Mission Message: " << str_data << std::endl;
      mission_status.mission_type = "Quick";

      // store search_chunk start position
      msg_components[3].erase(0, 1);
      position_strings = split(msg_components[3], delim);
      mission_status.lat = std::stod(position_strings[0]);
      mission_status.lon = std::stod(position_strings[1]);
      mission_status.alt = std::stof(position_strings[2]);

      // store search_chunk mission heading
      msg_components[4].erase(0, 1);
      mission_status.heading = std::stof(msg_components[4]);

      // store search_chunk field heading
      msg_components[5].erase(0, 1);
      mission_status.field_heading = std::stof(msg_components[5]);

      // store search_chunk distance
      msg_components[6].erase(0, 1);
      mission_status.distance = std::stof(msg_components[6]);

      // set flag to indicate new mission parameters received
      mission_status.changed_flag = true;
      vehicle_status.role = 0; // change role to quick
    }
    else if (!strcmp(msg_components[1].c_str(), "START") && valid_msg)
    {
      // NEWMSG,START
      std::cerr << "Handle Start Message: " << str_data << std::endl;
      mission_status.start = true;
    }
    else if (!strcmp(msg_components[1].c_str(), "STOP") && valid_msg)
    {
      // NEWMSG,STOP
      std::cerr << "Handle Stop Message: " << str_data << std::endl;
      mission_status.start = false;
    }
    else if (!strcmp(msg_components[1].c_str(), "POI") && valid_msg)
    {

      // Expect Msg: "NEWMSG,POI,QX,P35.1234 -120.5678,IX"
      delim = ' ';
      std::cerr << "Handle Point of Interest Message: " << str_data << std::endl;
      mission_status.mission_type = "Detailed";

      // store search_chunk start position
      msg_components[3].erase(0, 1);
      position_strings = split(msg_components[3], delim);
      mission_status.lat = std::stod(position_strings[0]);
      mission_status.lon = std::stod(position_strings[1]);
      msg_components[4].erase(0, 1);
      mission_status.poi_id = msg_components[4]; 
      vehicle_status.role = 1; // change role to detailed
      mission_status.changed_flag = true;
    }
    else if (!strcmp(msg_components[1].c_str(), "ROLE") && valid_msg)
    {

      std::cerr << "Handle Role Set Message: " << str_data << std::endl;

      // Expect Msg: "NEWMSG,ROLE,QX,RX
      msg_components[3].erase(0, 1); // store role information
      vehicle_status.role = std::stoi(msg_components[3].c_str());
      vehicle_status.role_changed = true;
      std::cerr << "Role Set as: " << vehicle_status.role << std::endl;
      PeeToCee.set_role(vehicle_status.role);
    }
    else
      valid_msg = false;
  }

  // If msg was invalid, output error msg to screen
  if (!valid_msg)
  {
    std::cerr << "Invalid Msg Read: " << str_data << std::endl;
    std::cerr << "Message ignored!" << std::endl;
  }
}

void UpdateGCS(XBEE::SerialXbee &xbee_interface, configContainer *configs)
{
  /* Function to write an update message to the GCS at GCS_MAC address
   *
   * The messge will have the form:
   * Q0,P35.300236 -120.661858 108.119000,SOnline,R0
   */
  XBEE::TransmitRequest frame_gcs(configs->gcs_mac);
  std::cerr << "Writing Msg: " << vehicle_status.gcs_update << std::endl
            << std::endl;
  frame_gcs.SetData(vehicle_status.gcs_update);
  xbee_interface.AsyncWriteFrame(&frame_gcs);
}

void PNavLoop(configContainer *configs, Log &logger)
{
  // TODO: Make a comms(RBP test mode which doesnt try to start the autopilot_interface)  test loop
  using namespace std::chrono;

  // Declare variables
  bool offboard = false;
  bool update_setpoint = false;
  bool cv_started = false;
  bool detailed_search_initialized = false;
  unsigned int ndx(0);
  int pattern(0);
  int cvGPS[2];
  int ballLat, ballLon;
  int poi_ctr = 0;
  std::string poi_id;
  coordLLA start_coordLLA;
  coordLocalNED startCoord;
  //time_t startTime;
  //time_t endTime;

  // Setup Autopilot interface
  #ifndef EMULATION
  Serial_Port serial_port(configs->uart_name.c_str(), configs->baudrate);
  Autopilot_Interface autopilot_interface(&serial_port);
  #endif

  std::cerr << "Before xbee initialization\n";
  // Setup Xbee serial interface and start reading
  // Use old XBEE library and COLONS instead of space
  XBEE::SerialXbee xbee_interface;
  xbee_interface.Connect();
  xbee_interface.ReadHandler = std::bind(&CallbackFunction, std::placeholders::_1);

  // declare variables for mavlink messages
  mavlink_set_position_target_local_ned_t sp;
  mavlink_set_position_target_local_ned_t previous_sp;
  mavlink_set_position_target_local_ned_t ip;
  mavlink_local_position_ned_t lpos;
  mavlink_global_position_int_t gpos;
  mavlink_position_target_local_ned_t tpos;
  Mavlink_Messages msgs;
  //    mavlink_position_target_local_ned_t tpos;

  // start timer for logging
  steady_clock::time_point t0_log, t0_heartbeat, t1_log, t1_heartbeat, t0_POI, t1_POI;
  #ifdef EMULATION
  steady_clock::time_point t0_flight_delay, t1_flight_delay;
  #endif
  flight_logger flt_log;

  #ifndef EMULATION
  serial_port_quit = &serial_port;
  autopilot_interface_quit = &autopilot_interface;
  #endif

  //  Start interface and take initial position
  #ifndef EMULATION
  serial_port.start();
  autopilot_interface.start();
  ip = autopilot_interface.initial_position;

  // TODO: Figure out how to correctly specify height
  startCoord << ip.x, ip.y, -configs->alt; // Assumes start position will be on ground
  #else
  startCoord << -1.60003, 0.358183, -configs->alt;
  #endif

  // instantiate a waypoints class
  waypoints mission_waypoints(configs);

  // Locate LNED frame origin
  // TODO: Implement method which checks LNED origin remains constant throughout flight

  #ifndef EMULATION
  msgs = autopilot_interface.current_messages;
  coordLocalNED LNED_0(msgs.local_position_ned.x,
                       msgs.local_position_ned.y,
                       msgs.local_position_ned.z);
  coordLLA LLA_0(msgs.global_position_int.lat * 1E-7 * M_PI / 180.0,
                 msgs.global_position_int.lon * 1E-7 * M_PI / 180.0,
                 msgs.global_position_int.alt * 1E-3);
  std::cerr << "nedx: " << msgs.local_position_ned.x << "nedy: " << msgs.local_position_ned.y << "nedz: " << msgs.local_position_ned.z << "\n";
  std::cerr << "lat: " << msgs.global_position_int.lat << "lon: " << msgs.global_position_int.lon << "alt: " << msgs.global_position_int.alt << "\n";
  #else
  coordLocalNED LNED_0(-1.60575, 0.366263, -1.17603);
  coordLLA LLA_0(CALPOLY_LAT * 1E-7 * M_PI / 180.0, CALPOLY_LON * 1E-7 * M_PI / 180.0, CALPOLY_ALT * 1E-3);
  #endif

  // TODO: Print origin too
  waypoints::FindOriginLocalNED(*configs, LNED_0, LLA_0);

  // Fly InputFile pattern if specified
  if (configs->pattern != 999)
    pattern = configs->pattern;
  else
    configs->pattern = RECTANGLE; // let GCS specify

  // Fly InputFile mission if specified
  if (configs->head != 999 && configs->dist != 0)
    mission_waypoints.SetWps(startCoord,
                             configs->head, configs->dist, configs->field_heading, pattern);

  if (pattern == CAM_ALTITUDE_TEST)
    configs->setpoint_tolerance = 1.0; // reduce setpoint tolerance for camera altitude test to make sure AV stops at each interval

  // store initial time for logging and writing heartbeat msgs
  t0_log = steady_clock::now();
  t0_heartbeat = steady_clock::now();

  #ifdef EMULATION
  t0_flight_delay = steady_clock::now();
  #endif

  ndx = 0;
  std::cerr << "Entering loop\n";

  // mainloop
  while (!PNav_shutdown)
  {
    if (vehicle_status.role_changed)
    {
      mission_waypoints.ClearMission();
      vehicle_status.role_changed = false;
    }
    // GCS reads are handled by the CallbackFunction
    // check if vehicle is in offboard mode
    #ifndef EMULATION
    offboard = ((0x00060000 & autopilot_interface.current_messages.heartbeat.custom_mode) == 393216);
    #else
    offboard = true;
    #endif

    // set current wp
    ndx = mission_waypoints.current_wp;
    //std::cerr << "ndx: " << ndx << " wps size: " << mission_waypoints.wps.size() << " pois size: " << mission_waypoints.POI.size() << std::endl;
    if (update_setpoint && (ndx < mission_waypoints.wps.size()) && !vehicle_status.role)
    {
      #ifndef EMULATION
      set_position(mission_waypoints.wps[ndx][0],
                   mission_waypoints.wps[ndx][1],
                   mission_waypoints.wps[ndx][2], sp);
      #else
      sp.x = mission_waypoints.wps[ndx][0];
      sp.y = mission_waypoints.wps[ndx][1];
      sp.z = mission_waypoints.wps[ndx][2];
      #endif

      //std::cerr << "[Quick Scan] Updating Setpoint " << ndx << " of " << mission_waypoints.wps.size() << std::endl;

      #ifndef EMULATION
      autopilot_interface.update_setpoint(sp);
      update_setpoint = false;
      #endif
    }
    else if (update_setpoint && (ndx < mission_waypoints.POI.size()) && vehicle_status.role)
    {
      #ifndef EMULATION
      set_position(mission_waypoints.POI[ndx][0],
                   mission_waypoints.POI[ndx][1],
                   mission_waypoints.POI[ndx][2], sp);
      #else
      sp.x = mission_waypoints.POI[ndx][0];
      sp.y = mission_waypoints.POI[ndx][1];
      sp.z = mission_waypoints.POI[ndx][2];
      #endif

      detailed_search_initialized = true;
      //std::cerr << "[Detailed Search] Updating Setpoint " << ndx << " of " << mission_waypoints.POI.size() << std::endl;

      #ifndef EMULATION
      autopilot_interface.update_setpoint(sp);
      update_setpoint = false;
      #endif

      std::cerr << "Moving to POI " << ndx << std::endl;
      vehicle_status.status = "Moving";
    }
    else if (ndx >= mission_waypoints.POI.size() && vehicle_status.role)
    {
      vehicle_status.status = "Idle";
    }
    else if (ndx >= mission_waypoints.wps.size() && mission_waypoints.wps.size() > 0 && !vehicle_status.role)
    {
      // Sends a message to GCS to update the role from quick scan to detailed search when it's done scanning
      vehicle_status.lat = gpos.lat * 1E-7;
      vehicle_status.lon = gpos.lon * 1E-7;
      vehicle_status.alt = gpos.alt * 1E-3;
      vehicle_status.role = 1;
      vehicle_status.gcs_update = "NEWMSG,ROLE,Q" + std::to_string(configs->quad_id) + ",P" +
                                  std::to_string(vehicle_status.lat) + " " +
                                  std::to_string(vehicle_status.lon) + " " + std::to_string(vehicle_status.alt) +
                                  ",S" + vehicle_status.status + ",R" + std::to_string(vehicle_status.role);
      std::cerr << "Updating GCS\n";
      UpdateGCS(xbee_interface, configs);
    }

    // check current location
    #ifndef EMULATION
    lpos = autopilot_interface.current_messages.local_position_ned;
    gpos = autopilot_interface.current_messages.global_position_int;
    tpos = autopilot_interface.current_messages.position_target_local_ned;
    #else
    if(mission_waypoints.wps.size() == 0) {
      lpos.x = LNED_0(0);
      lpos.y = LNED_0(1);
      lpos.z = LNED_0(2);

      previous_sp.x = lpos.x;
      previous_sp.y = lpos.y;
      previous_sp.z = lpos.z;

      gpos.lat = CALPOLY_LAT;
      gpos.lon = CALPOLY_LON;
      gpos.alt = CALPOLY_ALT;
    }
    else {
      t1_flight_delay = steady_clock::now();
      // get time difference in seconds
      auto time_diff = duration_cast<seconds>(t1_flight_delay - t0_flight_delay).count();

      lpos.x = previous_sp.x + time_diff * 4.4 * (sp.x - previous_sp.x);
      lpos.x = previous_sp.x + time_diff * 4.4 * (sp.y - previous_sp.y);
      lpos.x = previous_sp.x + time_diff * 4.4 * (sp.z - previous_sp.z);

      coordLocalNED lTemp(lpos.x, lpos.y, lpos.z);
      coordLLA gTemp = waypoints::LocalNEDtoLLA(*configs, lTemp, AngleType::DEGREES);

      gpos.lat = gTemp(0) * 1E7;
      gpos.lon = gTemp(1) * 1E7;
      gpos.alt = gTemp(2) * 1E3;
    }
    #endif

    if (!configs->debug_delay)
      std::this_thread::sleep_for(std::chrono::milliseconds(configs->debug_delay)); //Ask what this is for

    // if start command, set flag, (tell CV to start?)
    if (mission_status.start && mission_status.start != vehicle_status.start)
    {
      // TODO: Implement functionality for start/stop
      std::cerr << "Start commanded" << std::endl;
      vehicle_status.start = true;
      update_setpoint = true;
      if (!vehicle_status.role)
        vehicle_status.status = "Started";
    }

    // if stop command, set flag (tell CV to stop?)
    if (!mission_status.start && mission_status.start != vehicle_status.start)
    {
      std::cerr << "Stop commanded" << std::endl;
      vehicle_status.start = false;
      update_setpoint = false;
      vehicle_status.status = "Online";
      PeeToCee.set_CV_start(false);
      cv_started = false;
    }

    // if new search_chunk mission msg received, update Waypoint class
    if (mission_status.changed_flag)
    {
      std::cerr << "New Mission Received" << std::endl;
      mission_status.changed_flag = false;

      // calculate wp
      if (!vehicle_status.role)
      { // if Quick Search Mission
        start_coordLLA << (float)mission_status.lat, (float)mission_status.lon, configs->alt;
        start_coordLLA[0] = start_coordLLA[0] * M_PI / 180.0; // convert from deg2rad
        start_coordLLA[1] = start_coordLLA[1] * M_PI / 180.0;
        startCoord = waypoints::LLAtoLocalNED(*configs, start_coordLLA);
        startCoord[2] = ip.z - configs->alt;
        mission_waypoints.SetWps(startCoord, mission_status.heading, mission_status.distance, mission_status.field_heading, pattern);
        std::cerr << "New Search Chunk Set with Parameters: heading = " << mission_status.heading << ", field_heading: " << mission_status.field_heading << ", distance: " << mission_status.distance << std::endl;
        mission_waypoints.PlotWp(*configs, CoordFrame::LLA);
        mission_waypoints.PlotWp(*configs, CoordFrame::LOCAL_NED);
        std::cerr << std::endl;
      }
      else if (vehicle_status.role)
      { // append a POI waypoint
        start_coordLLA << (float)mission_status.lat, (float)mission_status.lon, configs->alt;
        start_coordLLA[0] = start_coordLLA[0] * M_PI / 180.0; // convert from deg2rad
        start_coordLLA[1] = start_coordLLA[1] * M_PI / 180.0;
        startCoord = waypoints::LLAtoLocalNED(*configs, start_coordLLA);
        startCoord[2] = ip.z - configs->alt;
        mission_waypoints.SetPOI(startCoord);
        std::cerr << "New POI Set\n";
        mission_waypoints.PlotPOI(*configs, CoordFrame::LLA);
        mission_waypoints.PlotPOI(*configs, CoordFrame::LOCAL_NED);
        std::cerr << std::endl;
      }
    }

    // if time change > heartbeat_freq, send a GPS location message
    t1_heartbeat = steady_clock::now();
    if ((((duration_cast<milliseconds>(t1_heartbeat - t0_heartbeat).count()) >
          (1 / configs->heartbeat_freq) * 1000)))
    {
      vehicle_status.LNED << lpos.x, lpos.y, lpos.z;
      vehicle_status.lat = gpos.lat * 1E-7;
      vehicle_status.lon = gpos.lon * 1E-7;
      vehicle_status.alt = gpos.alt * 1E-3;
      vehicle_status.gcs_update = "NEWMSG,UPDT,Q" + std::to_string(configs->quad_id) + ",P" +
                                  std::to_string(vehicle_status.lat) + " " +
                                  std::to_string(vehicle_status.lon) + " " + std::to_string(vehicle_status.alt) +
                                  ",S" + vehicle_status.status + ",R" + std::to_string(vehicle_status.role);
      //std::cerr << vehicle_status.gcs_update << std::endl;
      std::cerr << "Updating GCS\n";
      UpdateGCS(xbee_interface, configs);
      std::cerr << "Waypoint index " << ndx << " of " << mission_waypoints.wps.size() << "\n";
      t0_heartbeat = steady_clock::now();
    }

    // if time change > log->freq, log a message
    t1_log = steady_clock::now();
    if ((((duration_cast<milliseconds>(t1_log - t0_log).count()) >
          (1 / configs->log_freq) * 1000)))
    {

      #ifndef EMULATION
      flt_log.log(&autopilot_interface.current_messages);
      #endif
      t0_log = steady_clock::now();
    }
    
    //Should this go here?
    #ifndef EMULATION
    PeeToCee.set_GPS(gpos.lat, gpos.lon, gpos.alt, gpos.hdg, 
                      autopilot_interface.current_messages.attitude.pitch, 
                      autopilot_interface.current_messages.attitude.roll);
    #else
    PeeToCee.set_GPS(gpos.lat, gpos.lon, gpos.alt, gpos.hdg, 0, 0);
    #endif

    if (CeeToPee.CV_found())
    {
      //DOES THE NEW GPS GET PLACED HERE?
      ballLat = CeeToPee.get_ball_lat();
      ballLon = CeeToPee.get_ball_lon();
      if (vehicle_status.role)
      {
        mission_status.target_LLA << ballLat * 1E-7, ballLon * 1E-7, gpos.alt * 1E-3;
        vehicle_status.gcs_update = "NEWMSG,VLD,Q" + std::to_string(configs->quad_id) + ",P" +
                                    std::to_string(mission_status.target_LLA[0]) + " " +
                                    std::to_string(mission_status.target_LLA[1]) + " " + std::to_string(mission_status.target_LLA[2]) +
                                    ",S" + vehicle_status.status + ",R" + std::to_string(vehicle_status.role) + ",T" +
                                    std::to_string(mission_status.target_LLA[0]) + " " +
                                    std::to_string(mission_status.target_LLA[1]) + " " +
                                    std::to_string(mission_status.target_LLA[2]);
      }
      else
      {
        mission_status.target_LLA << ballLat * 1E-7, ballLon * 1E-7, gpos.alt * 1E-3;
        poi_id = (char)(configs->quad_id + 'A') + std::to_string(poi_ctr);
        vehicle_status.gcs_update = "NEWMSG,TGT,Q" + std::to_string(configs->quad_id) + ",P" +
                                    std::to_string(mission_status.target_LLA[0]) + " " +
                                    std::to_string(mission_status.target_LLA[1]) + " " + 
                                    std::to_string(mission_status.target_LLA[2]) +
                                    ",S" + vehicle_status.status + ",R" + std::to_string(vehicle_status.role) + ",T" +
                                    std::to_string(mission_status.target_LLA[0]) + " " +
                                    std::to_string(mission_status.target_LLA[1]) + " " +
                                    std::to_string(mission_status.target_LLA[2]) + ",I" + poi_id;
        poi_ctr++; 
      }
      CeeToPee.set_CV_found(false);
      UpdateGCS(xbee_interface, configs);
    }

    // if current location is within tolerance of target setpoint
    // increment current waypoint index
    //std::cerr << "difference x: " << fabs(lpos.x - sp.x) << "\n";
    if (offboard && (mission_waypoints.current_wp < mission_waypoints.wps.size()) &&
        (fabs(lpos.x - sp.x) < configs->setpoint_tolerance) &&
        (fabs(lpos.y - sp.y) < configs->setpoint_tolerance) &&
        (fabs(lpos.z - sp.z) < configs->setpoint_tolerance) && !vehicle_status.role
        && mission_status.start)

    //    if ((search_chunk_waypoints.current_wp < search_chunk_waypoints.wps.size()) &&
    //            (fabs(tpos.x - sp.x) < configs->setpoint_tolerance) &&
    //            (fabs(tpos.y - sp.y) < configs->setpoint_tolerance) &&
    //            (fabs(tpos.z - sp.z) < configs->setpoint_tolerance))
    {
      update_setpoint = true;
      mission_waypoints.current_wp++;
      // TODO comment
      previous_sp = sp;
      t0_flight_delay = steady_clock::now();
      if (!cv_started)
      { //Start CV when inside bounds
        cv_started = true;
        PeeToCee.set_CV_start(cv_started);
      }
      #ifndef EMULATION
      PeeToCee.set_GPS(gpos.lat, gpos.lon, gpos.alt, gpos.hdg, 
                       autopilot_interface.current_messages.attitude.pitch, 
                       autopilot_interface.current_messages.attitude.roll);
      #else
      PeeToCee.set_GPS(gpos.lat, gpos.lon, gpos.alt, gpos.hdg, 0, 0);
      #endif
    }
    else if (offboard && (mission_waypoints.current_wp < mission_waypoints.POI.size()) &&
        (fabs(lpos.x - sp.x) < configs->setpoint_tolerance) &&
        (fabs(lpos.y - sp.y) < configs->setpoint_tolerance) &&
        (fabs(lpos.z - sp.z) < configs->setpoint_tolerance) && vehicle_status.role
        && detailed_search_initialized && mission_status.start)
    {
      update_setpoint = true;
      mission_waypoints.current_wp++;
      // TODO comment
      previous_sp = sp;
      t0_flight_delay = steady_clock::now();
      vehicle_status.status = "Scanning";
      cv_started = true;
      PeeToCee.set_CV_start(cv_started);
      #ifndef EMULATION
      PeeToCee.set_GPS(gpos.lat, gpos.lon, gpos.alt, gpos.hdg, 
                       autopilot_interface.current_messages.attitude.pitch, 
                       autopilot_interface.current_messages.attitude.roll);
      #else
      PeeToCee.set_GPS(gpos.lat, gpos.lon, gpos.alt, gpos.hdg, 0, 0);
      #endif
      std::this_thread::sleep_for(std::chrono::milliseconds(2500));
      if (!CeeToPee.CV_found()) {
         vehicle_status.gcs_update = "NEWMSG,FP,Q" + std::to_string(configs->quad_id) + ",I" + mission_status.poi_id;
         UpdateGCS(xbee_interface, configs);
      }
      cv_started = false;
      while ((mission_waypoints.current_wp < mission_waypoints.POI.size()) &&
             (fabs(lpos.x - sp.x) < configs->setpoint_tolerance) &&
             (fabs(lpos.y - sp.y) < configs->setpoint_tolerance) &&
             (fabs(lpos.z - sp.z) < configs->setpoint_tolerance) && vehicle_status.role)
      {
        mission_waypoints.current_wp++;
      }

      // Wait until the location of next POI is processed before scanning again
      detailed_search_initialized = false;
    }

    // if current waypoint index is past end of waypoints vector, tell GCS and loiter
    // update stream to Pixhawk
  }
  //time(&endTime);
  //std::cerr << "Exiting Comms test loop" << difftime(startTime, endTime) << std::endl;
  PeeToCee.set_CV_start(false);
  #ifndef EMULATION
  autopilot_interface.stop();
  serial_port.stop();
  #endif
}
