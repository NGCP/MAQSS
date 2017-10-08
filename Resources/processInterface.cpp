/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   processInterface.cpp
 * Author: zhangh94
 * 
 * Created on August 31, 2016, 4:43 PM
 * 
 * Class to perform setup for MAQSS. Will parse commandline options + input file to create and open all necessary pipes. 
 */

#include <iostream>
#include <cstring>
#include <fstream>
#include <cstdlib>
#include <vector>

#include "processInterface.hpp"

#include <fcntl.h>
#include <stdio.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <signal.h>

processInterface::processInterface(configContainer *configs, int type) {
    /*
     * Handle creation and opening of pipes here
     */

    // TODO: Implement a better bidirectional communication (pty? stream V))
    // Open pipes based on which executable is requesting
    processType = type;
    // make pipe and throw error something unexpected happens (except file exists error)
    if (mkfifo(configs->pipe_CV_to_PNav.c_str(), 0666) && errno != 17) {
        std::cerr << "Error creating named pipe: " << errno << " - " << strerror(errno) << std::endl;
        exit(1);
    }

    // make cv_pipe2 so PNAV can write to CV
    if (mkfifo(configs->pipe_PNav_to_CV.c_str(), 0666) && errno != 17) {
        std::cerr << "Error creating named pipe: " << errno << " - " << strerror(errno) << std::endl;
        exit(1);
    }

    switch (processType) {
        case PNAV:
                configs->fd_CV_to_PNav = open(configs->pipe_CV_to_PNav.c_str(), O_RDONLY|O_NONBLOCK);
//                std::cerr << configs->pipe_CV_to_PNav << std::endl;
                configs->fd_PNav_to_CV = open(configs->pipe_PNav_to_CV.c_str(), O_WRONLY);
//                std::cerr << configs->pipe_PNav_to_CV << std::endl;
            break;

        case CV:
            configs->fd_CV_to_PNav = open(configs->pipe_CV_to_PNav.c_str(), O_WRONLY);
//            std::cerr << configs->pipe_CV_to_PNav << ", " << configs->fd_CV_to_PNav << std::endl;
            configs->fd_PNav_to_CV = open(configs->pipe_PNav_to_CV.c_str(), O_RDONLY);
//            std::cerr << configs->pipe_PNav_to_CV << ", " << configs->fd_PNav_to_CV << std::endl;
            std::cerr << "Error: " << errno << " - " << strerror(errno) << std::endl;
            break;

        default:
            std::cerr << "Invalid instantiation of processInterface" << std::endl;
            exit(1);
    }
}

int processInterface::openPipe(std::string pipeName, int opts) {
    int fd;
    fd = open(pipeName.c_str(), opts);
    return fd;
}

// TODO: Shoud read and write pipe methods specify a target from the enum? CV and PNav open 2 way pipe

void processInterface::writePipe(int fd, std::string msg) {
    write(fd, msg.c_str(), BUF_LEN);
}

void processInterface::printHelp() {
    std::cerr << "Help" << std::endl;
    std::cerr << "-f filename   Run PNav with inputfile \"filename\"" << std::endl;
    std::cerr << "--help        Print help" << std::endl;
}

void processInterface::cleanup(configContainer *configs) {
    std::cerr << "CLOSING PIPE" << std::endl;
    close(configs->fd_CV_to_PNav);
    close(configs->fd_PNav_to_CV);

    std::cerr << "UNLINKING PIPE" << std::endl;
    // Removes named pipes from directory if PNav process
    unlink(configs->pipe_CV_to_PNav.c_str());
    unlink(configs->pipe_PNav_to_CV.c_str());

}

processInterface::~processInterface() {
}
