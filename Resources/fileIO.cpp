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

configContainer fileIO::getConfig(int argc, char** argv) {
  int ndx(1);
  int ctr(0); // counter to track number of required inputs
  int c(0); // store char as int because RBPi stores unsigned chars
  char *tmpChars;
  char tmpStr[BUF_LEN];
  std::ifstream inputFile;
  configContainer configs;
  // parse command line arguments
  while (ndx < argc) // loop through all command line arguments
  {
    if (argv[ndx][0] == '-') {
      switch (argv[ndx][1]) {

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

        default:
          std::cerr << "Unrecognized command line argument"
                  << std::endl;
          ndx++;
          //                    printHelp(); // TODO: Make print help work here (return 0 and print in main?)
          break;
      }
    } else ndx++;
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
      if (!strcmp(tmpChars, "Version")) {
        ctr++; // increment counter for a required input
        configs.version = strtok(NULL, "  \n");
      } else if (!strcmp(tmpChars, "pipe_CV_to_PNav")) {
        ctr++;
        configs.pipe_CV_to_PNav = strtok(NULL, " \n");
      } else if (!strcmp(tmpChars, "pipe_PNav_to_CV")) {
        ctr++;
        configs.pipe_PNav_to_CV = strtok(NULL, " \n");
      } else if (!strcmp(tmpChars, "uart_name")) {
        ctr++;
        configs.uart_name = strtok(NULL, " \n");
      } else if (!strcmp(tmpChars, "Quad")) {
        ctr++;
        configs.quad_id = (int)(strtok(NULL, " \n")[0] - 'A');
      }// Known config params but not required
      else if (!strcmp(tmpChars, "baudrate")) configs.baudrate = std::stoi(strtok(NULL, " \n"));

      else if (!strcmp(tmpChars, "heartbeat_freq")) configs.heartbeat_freq = std::stof(strtok(NULL, " \n"));

      else if (!strcmp(tmpChars, "cam_FOV_v")) configs.cam_FOV_v = std::stoi(strtok(NULL, " \n"));

      else if (!strcmp(tmpChars, "cam_FOV_h")) configs.cam_FOV_h = std::stoi(strtok(NULL, " \n"));

      else if (!strcmp(tmpChars, "npoints")) configs.npoints = std::stoi(strtok(NULL, " \n"));

      else if (!strcmp(tmpChars, "pattern")) configs.pattern = std::stoi(strtok(NULL, " \n"));

      else if (!strcmp(tmpChars, "dist")) configs.dist = std::stoi(strtok(NULL, " \n"));

      else if (!strcmp(tmpChars, "head")) configs.head = std::stoi(strtok(NULL, " \n"));

      else if (!strcmp(tmpChars, "alt")) configs.alt = std::stoi(strtok(NULL, " \n"));

      else if (!strcmp(tmpChars, "video_Stabilization")) configs.video_Stabilization = (std::stoi(strtok(NULL, " \n")) ? true : false);

      else if (!strcmp(tmpChars, "cam_Test")) configs.cam_Test = (std::stoi(strtok(NULL, " \n")) ? true : false);

      else if (!strcmp(tmpChars, "cap_Freq")) configs.cap_Freq = std::stof(strtok(NULL, " \n"));

      else if (!strcmp(tmpChars, "cam_Width")) configs.cam_Width = std::stoi(strtok(NULL, " \n"));

      else if (!strcmp(tmpChars, "cam_Height")) configs.cam_Height = std::stoi(strtok(NULL, " \n"));

      else if (!strcmp(tmpChars, "log")) configs.log = (std::stoi(strtok(NULL, " \n")) ? true : false);

      else if (!strcmp(tmpChars, "log_freq")) configs.log_freq = std::stof(strtok(NULL, " \n"));

      else if (!strcmp(tmpChars, "log_level")) configs.log_level = std::stof(strtok(NULL, " \n"));

      else if (!strcmp(tmpChars, "gcs_mac")) configs.gcs_mac = std::stof(strtok(NULL, " \n"));

      else if (!strcmp(tmpChars, "radio_mac")) configs.gcs_mac = std::stof(strtok(NULL, " \n"));

      else if (!strcmp(tmpChars, "debug_delay")) configs.debug_delay = std::stoi(strtok(NULL, " \n"));

      else if (!strcmp(tmpChars, "field_heading")) configs.field_heading = std::stof(strtok(NULL, " \n"));

      else if (!strcmp(tmpChars, "setpoint_tolerance")) configs.setpoint_tolerance = std::stof(strtok(NULL, " \n"));
        // Misc config params
      else {
        // miscParams are stored in a map where the key is the InputFile param name, and the value is a void pointer which points to the data after the '='
        configs.misc_Params[tmpStr] = (void*) (strtok(NULL, " \n"));
      }

    }
    // check if EOF
    c = inputFile.peek();
  }

  inputFile.close();
  if (ctr < 4) {
    std::cerr << "Not all required input params provided" << std::endl;
    exit(1);
  }
  return configs;
}

void fileIO::printConfig(configContainer *configs) {
  std::cerr << "Printing Configuration:" << std::endl;
  std::cerr << "MAQSS Version: " << configs->version << std::endl;
  std::cerr << "Quad: " << configs->quad_id << std::endl;
  std::cerr << std::endl;

  std::cerr << "Interface Parameters:" << std::endl;
  std::cerr << "CV write to PNav Pipe: " << configs->pipe_CV_to_PNav << std::endl;
  std::cerr << "PNav write to CV Pipe: " << configs->pipe_PNav_to_CV << std::endl;
  std::cerr << "UART Device Path: " << configs->uart_name << std::endl;
  std::cerr << "UART baudrate: " << configs->baudrate << std::endl;
  std::cerr << std::endl;

  std::cerr << "Mission Parameters:" << std::endl;
  std::cerr << "Camera Test Mode: " << configs->cam_Test << std::endl;
  std::cerr << "Camera Test Capture Frequency: " << configs->cap_Freq << std::endl;
  std::cerr << "Altitude Parameter: " << configs->alt << std::endl;
  std::cerr << "Distance Parameter: " << configs->dist << std::endl;
  std::cerr << "Heading Parameter: " << configs->head << std::endl;
  std::cerr << "Field Heading Parameter: " << configs->field_heading << std::endl;
  std::cerr << "Number of SetPoints: " << configs->npoints << std::endl;
  std::cerr << "Setpoint Tolerance: " << configs->setpoint_tolerance << std::endl;
  std::cerr << "Pattern Enum: " << configs->pattern << std::endl;
  std::cerr << std::endl;

  std::cerr << "Camera Parameters: " << std::endl;
  std::cerr << "Camera Vertical FOV: " << configs->cam_FOV_v << std::endl;
  std::cerr << "Camera Horizontal FOV: " << configs->cam_FOV_h << std::endl;
  std::cerr << "Image Width: " << configs->cam_Width << std::endl;
  std::cerr << "Image Height: " << configs->cam_Height << std::endl;
  std::cerr << std::endl;

  std::cerr << "Logging Parameters" << std::endl;
  std::cerr << "Logging Enabled: " << configs->log << std::endl;
  std::cerr << "Logging Frequency: " << configs->log_freq << std::endl;
  std::cerr << "Logging Level: " << configs->log_level << std::endl;

  std::cerr << "GCS MAC Address: " << configs->gcs_mac << std::endl;

  std::cerr << "End Configuration\n" << std::endl;

}

fileIO::~fileIO() {
}
