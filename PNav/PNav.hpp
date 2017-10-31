#ifndef PNAV_HPP
#define PNAV_HPP

// ------------------------------------------------------------------------------
//   Quit Signal Handler
// ------------------------------------------------------------------------------
// this function is called when you press Ctrl-C
#define GCS_MAC 0x0013A20040F8064C

void PNavLoop(configContainer *configs);

#endif