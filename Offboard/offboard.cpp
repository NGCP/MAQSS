// C++ headers
#include <iostream>
#include <string>
#include <thread>
#include <mutex>

// C headers
#include <signal.h>

// Custom headers
#include "fileIO.hpp"
#include "offboard.hpp"
#include "PNav.hpp"
#include "CV.hpp"
#include "log.hpp"

/*
 * CeeToPee and PeeToCee are used to communicate safely between the CV thread
 * and the navigation thread.
 */

//PNav_to_CV functions
PNav_to_CV::PNav_to_CV() {
    CV_start_ = false;
    CV_exit_ = false;
    role = QUICK_SEARCH; //Auto set to quicksearch
}

bool PNav_to_CV::CV_start() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return CV_start_;
}

bool PNav_to_CV::CV_exit() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return CV_exit_;
}

int PNav_to_CV::get_role() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return role;
}

int PNav_to_CV::get_lat() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return lat;
}

int PNav_to_CV::get_lon() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return lon;
}

int PNav_to_CV::get_height() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return height;
}

int PNav_to_CV::get_yaw() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return yaw;
}

float PNav_to_CV::get_pitch() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return pitch;
}

float PNav_to_CV::get_roll() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return roll;
}

void PNav_to_CV::set_CV_start(bool set) {
    std::lock_guard<std::mutex> lock(mutex_);
    CV_start_ = set;
}

void PNav_to_CV::set_CV_exit(bool set) {
    std::lock_guard<std::mutex> lock(mutex_);
    CV_exit_ = set;
}

void PNav_to_CV::set_role(int set) {
    std::lock_guard<std::mutex> lock(mutex_);
    role = set;
}
/*
 * Latitude, Longitude are int's actual positions need to be multiplied
 * by 1E-7 for accurate position
 */
void PNav_to_CV::set_GPS(int lat, int lon, int height, int yaw, float pitchDeg, float rollDeg) {
    std::lock_guard<std::mutex> lock(mutex_);
    this->lat = lat;
    this->lon = lon;
    this->yaw = yaw;
    pitch = pitchDeg;
    roll = rollDeg;

}
//CV_to_PNav functions
CV_to_PNav::CV_to_PNav() {
    CV_found_ = false;
}

bool CV_to_PNav::CV_found() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return CV_found_;
}

int CV_to_PNav::get_ball_lat() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return latitude_;
}

int CV_to_PNav::get_ball_lon() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return longitude_;
}

void CV_to_PNav::set_CV_found(bool set) {
    std::lock_guard<std::mutex> lock(mutex_);
    CV_found_ = set;
}

void CV_to_PNav::set_ball_lat(int lat) {
    std::lock_guard<std::mutex> lock(mutex_);
    latitude_ = lat;
}

void CV_to_PNav::set_ball_lon(int lon) {
    std::lock_guard<std::mutex> lock(mutex_);
    longitude_ = lon;
}

void CV_to_PNav::CV_lock() {
    mutex_.lock();
}

void CV_to_PNav::CV_unlock() {
    mutex_.unlock();
}


// TODO: Have karthik fix my makefile

PNav_to_CV PeeToCee;
CV_to_PNav CeeToPee;
configContainer *configPointer;

void quit_handler(int sig) {
  std::cerr << "\nTERMINATING AT USER REQUEST\n\n";
  //PNav cleanup
  if (configPointer->cam_Test) {
    PeeToCee.set_CV_start(false);
    CV_call_stop();
    std::cerr << "Stopping CV\n";
  }
  else {
     PNav_call_stop();
     CV_call_stop();
     std::cerr << "Stopping PNAV and CV\n";
  }
}


int main(int argc, char **argv) {
    unsigned int nCaptures;
    configContainer configs;
    
    if (argc > 1)
        configs = fileIO::getConfig(argc, argv);
    else {
        std::cerr << "Must specify command line arguments" << std::endl;
        exit(EXIT_FAILURE);
    }
    configPointer = &configs;

    Log Logger(configs.log_level);
    Logger.log(3, "Offboard started");

    fileIO::printConfig(&configs);
    nCaptures = 0;
    signal(SIGINT, quit_handler);

    if (configs.cam_Test) {
        //testLoop(nCaptures, configPointer);
    }
    else{
        CeeToPee.set_CV_found(false);
        std::thread CV_thread(frameLoop, std::ref(nCaptures), std::ref(configPointer), std::ref(Logger));
        std::thread PNav_thread(PNavLoop, std::ref(configPointer), std::ref(Logger));
        CV_thread.join();
        PNav_thread.join();
    }
    exit(EXIT_SUCCESS);
}
