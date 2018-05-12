/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   processInterface.hpp
 * Author: zhangh94
 *
 * Created on August 31, 2016, 4:43 PM
 */

#ifndef processInterface_HPP
#define processInterface_HPP

#include "configContainer.hpp"
#define BUF_LEN 1024
enum processes {
    PNAV,
    CV,
    
    NUM_PROCESSES
};


class processInterface {
public:
    // pass in configs from InputFile and specify process type based on processes enum
    processInterface(configContainer *configs, int type);
    

    int openPipe(std::string pipeName, int opts);
    void writePipe(int fd, std::string msg);
    void printHelp();
    void cleanup(configContainer *configs);
 
    // TODO: Move into a file reader class
    // TODO: make these static methods (will instantate a data container class and return)
    virtual ~processInterface();
    
    // TODO: make data container class
    // TODO: Parse for required inputs (err if missing), store extra inputs
    
private:
    
    int processType;
    
};

#endif /* processInterface_HPP */
