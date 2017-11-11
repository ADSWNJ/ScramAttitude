// ==============================================================
//
//	ScramAttitude (Core Persistence)
//	================================
//
//	Copyright (C) 2016-2017	Andrew (ADSWNJ) Stokes
//                   All rights reserved
//
//	See ScramAttitude.cpp
//
// ==============================================================

#include "ScramAttitude_GCore.hpp"
#include "ScramAttitude_VCore.hpp"

ScramAttitude_GCore::ScramAttitude_GCore() {
  return;
}

ScramAttitude_GCore::~ScramAttitude_GCore() {
  return;
}


void ScramAttitude_GCore::corePreStep(double simT,double simDT,double mjd) {
  if (coreSimT == 0) {
    coreSimT = simT;
    return;
  }
  if (coreSimT == simT) return;

  if (P.firstVC() == NULL) return; // No vessels interested in ScramAttitude yet

  coreSimDT = simT - coreSimT;
  coreSimT = simT;
  //sprintf(oapiDebugString(),"GCORE PRESTEP: %15.15f", coreSimDT);

  // Once per update - call vessel corePreSteps
  for (ScramAttitude_VCore* VC = (ScramAttitude_VCore*) P.firstVC(); VC != NULL; VC = (ScramAttitude_VCore*) P.nextVC()) {
    VC->corePreStep(coreSimT, coreSimDT, mjd);
  }

}
