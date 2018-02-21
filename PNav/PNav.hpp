#ifndef PNAV_HPP
#define PNAV_HPP

// ------------------------------------------------------------------------------
//   Quit Signal Handler
// ------------------------------------------------------------------------------
// this function is called when you press Ctrl-C
#define GCS_MAC 0x0013A200409BD79C

void PNav_call_stop();
void PNavLoop(configContainer *configs);

#endif