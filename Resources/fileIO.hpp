/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   fileIO.hpp
 * Author: zhangh94
 *
 * Created on September 12, 2016, 6:30 PM
 */

#ifndef FILEIO_HPP
#define FILEIO_HPP

#include "configContainer.hpp"

class fileIO {
public:
    fileIO();
    static configContainer getConfig(int argc, char** argv);
    static void printConfig(configContainer *configs);
    
//    static readFile();
    virtual ~fileIO();
private:

    int fp;
};

#endif /* FILEIO_HPP */

