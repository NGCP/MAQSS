/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   fileIO.cpp
 * Author: zhangh94
 * 
 * Created on September 12, 2016, 6:30 PM
 */

#include <fstream>
#include <iostream>
#include <map>
#include <string>
#include <string.h>


#include "processInterface.hpp"
#include "fileIO.hpp"

fileIO::fileIO() {
    
}

configContainer fileIO::getConfig(int argc, char** argv)
{
    int ndx(1);
    int ctr(0); // counter to track number of required inputs
    int c(0); // store char as int because RBPi stores unsigned chars
    char *tmpChars;
    char  tmpStr[BUF_LEN];
    std::ifstream inputFile;
    configContainer configs;
    // parse command line arguments
    while (ndx < argc) // loop through all command line arguments
    {
        if (argv[ndx][0] == '-')
        {
            switch (argv[ndx][1])
            {

                case 'f': // take in inputfile
                    if (argv[ndx + 1] == NULL) // TODO: Improve this for more cases
                    {
                        std::cerr << "Must specify input file after -f argument" << std::endl;
                        exit(1);
                    }
                    configs.input_filename = argv[ndx + 1];
                    ndx++;
                    break;
                    
                    // TODO: Reimplement --help option
//                case '-': // asking for help
//                    if (!strcmp(argv[ndx], "--help")) printHelp();
//                    ndx++;
//                    break;

                default : 
                    std::cerr <<  "Unrecognized command line argument"
                        << std::endl;
                    ndx++;
//                    printHelp(); // TODO: Make print help work here (return 0 and print in main?)
                    break;
            }    
        }
        else ndx++;
    }  

    // TODO: Make this handling case insensitive
    inputFile.open(configs.input_filename.c_str());

    std::cerr << "InputFile Opened: " << configs.input_filename << std::endl;
    
    while (c != EOF) {
        // get all lines from inputFile
        inputFile.getline(tmpStr, BUF_LEN, '\n');
        
        if (strlen(tmpStr) > 1 && tmpStr[0] != '#') { // if line isnt empty
            tmpChars = strtok(tmpStr, "=");
            
            // Required config params
            if (!strcmp(tmpChars, "Version") ) 
            {
                ctr++; // increment counter for a required input
                configs.version = strtok(NULL,"  \n");
            }
            else if (!strcmp(tmpChars, "cv_pipe"))
            {
                ctr++;
                configs.cv_pipe = strtok(NULL, " \n");
            }
            else if (!strcmp(tmpChars, "cv_pipe2"))
            {
                ctr++;
                configs.cv_pipe2 = strtok(NULL, " \n");
            }
            else if (!strcmp(tmpChars, "uart_name"))
            {
                ctr++;
                configs.uart_name = strtok(NULL, " \n");
            }
            
            // Known config params but not required
            else if(!strcmp(tmpChars, "baudrate")) configs.baudrate = std::stoi(strtok(NULL, " \n"));
            
            else if(!strcmp(tmpChars, "camFOV_v")) configs.camFOV_v = std::stoi(strtok(NULL, " \n"));
            
            else if(!strcmp(tmpChars, "camFOV_h")) configs.camFOV_h = std::stoi(strtok(NULL, " \n"));
            
            else if(!strcmp(tmpChars, "npoints")) configs.npoints = std::stoi(strtok(NULL, " \n"));
            
            else if(!strcmp(tmpChars, "pattern")) configs.pattern = std::stoi(strtok(NULL, " \n"));
            
            else if(!strcmp(tmpChars, "dist")) configs.dist = std::stoi(strtok(NULL, " \n"));
            
            else if(!strcmp(tmpChars, "head")) configs.head = std::stoi(strtok(NULL, " \n"));
            
            else if(!strcmp(tmpChars, "videoStabilization")) configs.videoStabilization = (std::stoi(strtok(NULL, " \n")) ? true: false);
            
            else if(!strcmp(tmpChars, "camTest")) configs.camTest = (std::stoi(strtok(NULL, " \n")) ? true: false);
            
            // Misc config params
            else
            {
                // miscParams are stored in a map where the key is the InputFile param name, and the value is a void pointer which points to the data after the '='
                configs.miscParams[tmpStr] = (void*)(strtok(NULL," \n"));
            }
            
        }
        // check if EOF
        c = inputFile.peek();
    }
    
    inputFile.close();
    if (ctr < 3) {
        std::cerr << "Not all required input params provided" << std::endl;
        exit(1);
    }
    return configs;
}

fileIO::~fileIO() {
}

