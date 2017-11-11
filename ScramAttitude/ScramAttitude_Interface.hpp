// ==============================================================
//
//	ScramAttitude (Orbiter Interface)
//	============================
//
//	Copyright (C) 2016-2017	Andrew (ADSWNJ) Stokes
//                   All rights reserved
//
//	See ScramAttitude.cpp
//
//

#ifndef SCRAM_ATT_INTERFACE
#define SCRAM_ATT_INTERFACE

#include "windows.h"
#include "orbitersdk.h"
#include "ScramAttitude.hpp"


class ScramAttitude_Interface : public oapi::Module {
public:
  ScramAttitude_Interface(HINSTANCE hDLL);
  ~ScramAttitude_Interface();
  void clbkSimulationStart(RenderMode mode);
  void clbkSimulationEnd();
  void clbkPreStep (double simt, double simdt, double mjd);
  //void clbkPostStep (double simt, double simdt, double mjd);
  void clbkDeleteVessel (OBJHANDLE hVessel);
  static int MsgProc (UINT msg, UINT mfd, WPARAM wparam, LPARAM lparam);

};
#endif // SCRAM_ATT_INTERFACE