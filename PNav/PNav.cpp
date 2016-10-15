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

int mainLoop(processInterface *PNav, configContainer *configs) {
    
    using namespace std::chrono;
    
    // Declare variables
    bool set_point_reached = false;
    unsigned int ndx(0);
    int set_tolerance = 3;
    int pattern(0);
    char tmp[BUF_LEN];
    coord startCoord;
    std::string cv_msg = "Start";

    // Setup Autopilot interface
    Serial_Port serial_port(configs->uart_name.c_str(), configs->baudrate);
    Autopilot_Interface autopilot_interface(&serial_port);

    // instantiate a waypoints class
    waypoints searchChunk(configs);

    // declare variables for mavlink messages
    mavlink_set_position_target_local_ned_t sp;
    mavlink_set_position_target_local_ned_t ip;
    mavlink_local_position_ned_t lpos;
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

    // Setup GCS interface
    if (configs->cam_Test) {
        std::cerr << "Camera test mode, PNav idling" << std::endl;
        while (true);
    }

    //  Start interface and take initial position
    serial_port.start();
    autopilot_interface.start();
    ip = autopilot_interface.initial_position;
    startCoord = {ip.x, ip.y, ip.z - configs->alt};

    // TODO: figure out how to orient northwards (point quad north first)

    // Fly InputFile pattern if specified
    if (configs->pattern != 999) pattern = configs->pattern;
    else {
    } // let GCS specify

    // Fly InputFile mission if specified
    if (configs->head != 999 && configs->dist != 0) searchChunk.setWps(startCoord, configs->head, configs->dist, pattern);
    else searchChunk.setWps(startCoord, 330, 50, FIG8);

    if (pattern == CAM_ALTITUDE_TEST) set_tolerance = 0.5; // reduce setpoint tolerance for camera altitude test to make sure AV stops at each interval

    std::cerr << "Current IP: [" << ip.x << ", " << ip.y << "," << ip.z << "]" << std::endl;
    std::cerr << "Begin Streaming setpoints:" << std::endl;
    std::cerr << "sp = [" << std::endl;
    for (ndx = 0; ndx < searchChunk.wps.size(); ndx++) {
        t0 = steady_clock::now();
        set_position(searchChunk.wps[ndx][0], searchChunk.wps[ndx][1], searchChunk.wps[ndx][2], sp);
        autopilot_interface.update_setpoint(sp);
        set_point_reached = false;

        PNav->writePipe(configs->fd_PNav_to_CV, cv_msg);

        std::cerr << ndx + 1 << ", " << searchChunk.wps[ndx][0] << ", " << searchChunk.wps[ndx][1] << ", " << searchChunk.wps[ndx][2] << "; " << std::endl;
        while (!set_point_reached) {

            //            tpos = autopilot_interface.current_messages.position_target_local_ned;
            //            std::cerr << tpos.x << ", " << tpos.y << ", " << tpos.z << std::endl;
            //            sleep(1);
            //            if (abs(tpos.x - sp.x) < set_tolerance && abs(tpos.y - sp.y) < set_tolerance && abs(tpos.z - sp.z) < set_tolerance) {
            //                set_point_reached = true;
            //                break;
            //            }

            lpos = autopilot_interface.current_messages.local_position_ned;
            //            std::cerr << "LPos"
            //            if (abs(lpos.x - sp.x) < setTolerance && abs(lpos.y - sp.y) < setTolerance && abs(lpos.z - sp.z) < setTolerance) {

            // TODO: See if removing Z tolerance helps
            if (abs(lpos.x - sp.x) < set_tolerance && abs(lpos.y - sp.y) < set_tolerance) {
                set_point_reached = true;
                break;
            }
            
            // Check timer to log data
            t1 = steady_clock::now();
            if (configs->log && (((duration_cast<milliseconds>(t1 - t0).count()) > (1/configs->log_freq) * 1000))) {
                flt_log.log(&autopilot_interface.current_messages);
                t0 = steady_clock::now();
            }
            ndx = ((ndx == configs->npoints - 1) ? 0 : ndx);
        }
        std::cerr << "Waiting for msg from Cv" << std::endl;
        while (!read(configs->fd_CV_to_PNav, tmp, BUF_LEN)) {
        }
        std::cerr << "Read msg from Cv: " << tmp << std::endl;
    }
    std::cerr << "];" << std::endl;
    std::cerr << "plot(sp(:,2),sp(:,3),'-x')\nhold all\n axis equal\ngrid on\nplot(sp(1,2),sp(1,3),'or')" << std::endl;

    cv_msg = "Exit";
    PNav->writePipe(configs->fd_PNav_to_CV, cv_msg);

    // switch through (read from CV, check setpoint, check GCS comms, write GPS to GCS if past 1s)
    //    while (1)
    {

        // if read(gcsInterface) has information
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
