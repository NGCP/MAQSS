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

    //    printConfig(configs);

    // TODO: Implement a better bidirectional communication (pty? stream V))
    // Open pipes based on which executable is requesting
    processType = type;
    // make pipe and throw error something unexpected happens (except file exists error)
    if (mkfifo(configs->cv_pipe.c_str(), 0666) && errno != 17) {
        std::cerr << "Error creating named pipe: " << errno << " - " << strerror(errno) << std::endl;
        exit(1);
    }

    // make cv_pipe2 so PNAV can write to CV
    if (mkfifo(configs->cv_pipe2.c_str(), 0666) && errno != 17) {
        std::cerr << "Error creating named pipe: " << errno << " - " << strerror(errno) << std::endl;
        exit(1);
    }
    
    switch (processType) {
        case PNAV:

            configs->cv_fd = open(configs->cv_pipe.c_str(), O_RDONLY);
            std::cerr << configs->cv_pipe << std::endl;
            configs->cv_fd2 = open(configs->cv_pipe2.c_str(), O_WRONLY);
            std::cerr << configs->cv_pipe2 << std::endl;
            break;

        case CV:
            configs->cv_fd = open(configs->cv_pipe.c_str(), O_WRONLY);
            std::cerr << configs->cv_pipe << ", " << configs->cv_fd << std::endl;
            configs->cv_fd2 = open(configs->cv_pipe2.c_str(), O_RDONLY);
            std::cerr << configs->cv_pipe2 << ", " << configs->cv_fd2 << std::endl;
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
//
//void processInterface::handleOptions(int argc, char** argv) {
//    int ndx(1);
//    char c(0);
//    char tmpStr[BUF_LEN];
//    std::ifstream inputFile;
//
//    // Check which process is running
//    if (!strcmp(argv[0], "./PNav")) processType = PNAV;
//    else if (!strcmp(argv[0], "./CV")) processType = CV;
//    
//    // parse command line arguments
//    while (ndx < argc) // loop through all command line arguments
//    {
//
//        if (argv[ndx][0] == '-')
//        {
//            switch (argv[ndx][1])
//            {
//
//                case 'f': // take in inputfile
//                    // TODO: Handle if user doesnt input fname after -f
//                    
//                    input_filename = argv[ndx + 1];
//                    ndx++;
//                    // TODO: Handle fileopen error
//                    // TODO: Make this handling case insensitive
//                    inputFile.open(input_filename.c_str());
//                    
//                    // TODO: strTok c method
//                    // TODO: Use a map to read from line (use a one to one map)
//                    while (c != EOF) 
//                    {
//                        inputFile.getline(tmpStr, BUF_LEN,'=');
//                        
//                        // create function to handle InputFile params
//                        // Pass it in a configuration class
//                        if (!strcmp(tmpStr, "cv_pipe")) 
//                        {
//                            inputFile.getline(tmpStr, BUF_LEN, '\n');
//                            cv_pipe = tmpStr;
//                        }
//                        else if (!strcmp(tmpStr, "MajorVersion"))
//                        {
//                            
//                            // TODO: make this less ugly
//                            inputFile.getline(tmpStr, BUF_LEN, '\n');
//                            version = (version = tmpStr) + ".";
//                            inputFile.getline(tmpStr, BUF_LEN, '=');
//                            inputFile.getline(tmpStr, BUF_LEN, '\n');
//                            version += tmpStr;
//                        }
//                        
//                        else if (!strcmp(tmpStr, "uart_name"))
//                        {
//                            inputFile.getline(tmpStr, BUF_LEN, '\n');
//                            uart_name = tmpStr;
//                        }
//                        
//                        else if (!strcmp(tmpStr, "baudrate"))
//                        {
//                            inputFile.getline(tmpStr, BUF_LEN, '\n');
//                            baudrate = std::stoi(tmpStr);
//                        }
//                        
//                        else if (!strcmp(tmpStr, "camFOV_v"))
//                        {
//                            inputFile.getline(tmpStr, BUF_LEN, '\n');
//                            camFOV_v = std::stoi(tmpStr);
//                            inputFile.getline(tmpStr, BUF_LEN, '=');
//                            inputFile.getline(tmpStr, BUF_LEN, '\n');
//                            camFOV_h = std::stoi(tmpStr);
//                        }
//                        // check if EOF
//                        c = inputFile.peek();
//                    }
//
//                    inputFile.close();
//                    break;
//
//                case '-': // asking for help
//                    if (!strcmp(argv[ndx], "--help")) printHelp();
//                    ndx++;
//                    break;
//
//                default : 
//                    std::cerr <<  "Unrecognized command line argument"
//                        << std::endl;
//                    ndx++;
//                    printHelp();
//                    break;
//            }    
//        }
//        else ndx++;
//    }
//}

void processInterface::printConfig(configContainer *configs) {
    std::cout << "MAQSS Configuration" << std::endl;
    std::cout << "Version: " << configs->version << std::endl;
    std::cout << "Connecting to Pixhawk at " << configs->baudrate << " on: " << configs->uart_name << std::endl;
}

void processInterface::printHelp() {
    std::cerr << "Help" << std::endl;
    std::cerr << "-f filename   Run PNav with inputfile \"filename\"" << std::endl;
    std::cerr << "--help        Print help" << std::endl;
}

void processInterface::cleanup(configContainer *configs) {
    std::cerr << "CLOSING PIPE" << std::endl;
    close(configs->cv_fd);
    close(configs->cv_fd2);

    std::cerr << "UNLINKING PIPE" << std::endl;
    // Removes named pipes from directory if PNav process
    unlink(configs->cv_pipe.c_str());
    unlink(configs->cv_pipe2.c_str());

    //    if (processType == PNAV) {
    //        unlink(configs->cv_pipe.c_str());
    //        unlink(configs->cv_pipe2.c_str());
    //    }
}

processInterface::~processInterface() {
}

