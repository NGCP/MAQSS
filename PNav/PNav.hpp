#ifndef PNAV_HPP
#define PNAV_HPP

#include "log.hpp"

// ------------------------------------------------------------------------------
//   Quit Signal Handler
// ------------------------------------------------------------------------------
// this function is called when you press Ctrl-C
#define GCS_MAC 0x0013A20040F8064C

void PNav_call_stop();
void PNavLoop(configContainer *configs, Log &logger);

#endif