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

//PNav_to_CV functions
PNav_to_CV::PNav_to_CV() {
    CV_start_ = false;
    CV_exit_ = false;
    role = 0; //Auto set to quicksearch
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

//CV_to_PNav functions
CV_to_PNav::CV_to_PNav() {
    CV_found_ = false;
}

bool CV_to_PNav::CV_found() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return CV_found_;
}

void CV_to_PNav::set_CV_found(bool set) {
    std::lock_guard<std::mutex> lock(mutex_);
    CV_found_ = set;
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
    configContainer configs;
    unsigned int nCaptures;
    Log Logger;

    if (argc > 1)
        configs = fileIO::getConfig(argc, argv);
    else {
        std::cerr << "Must specify command line arguements" << std::endl;
        exit(EXIT_FAILURE);
    }
    fileIO::printConfig(&configs);
    configPointer = &configs;
    nCaptures = 0;
    signal(SIGINT, quit_handler);

    if (configs.cam_Test) {
        testLoop(nCaptures, configPointer);
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

