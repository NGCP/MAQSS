#ifndef PNAV_HPP
#define PNAV_HPP

#include "log.hpp"

// ------------------------------------------------------------------------------
//   Quit Signal Handler
// ------------------------------------------------------------------------------
// this function is called when you press Ctrl-C
void PNav_call_stop();
void PNavLoop(configContainer *configs, Log &logger);

#endif