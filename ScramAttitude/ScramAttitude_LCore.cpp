// ==========================================================================
//
//	ScramAttitude (Local (Vessel+MFD Panel) Core Persistence)
//	=========================================================
//
//	Copyright (C) 2016-2017	Andrew (ADSWNJ) Stokes
//                   All rights reserved
//
//	See ScramAttitude.cpp
//
// ==========================================================================

#include "ScramAttitude_GCore.hpp"
#include "ScramAttitude_VCore.hpp"
#include "ScramAttitude_LCore.hpp"

ScramAttitude_LCore::ScramAttitude_LCore(VESSEL *vin, UINT mfdin, ScramAttitude_GCore* gcin) {
  GC = gcin;
  v = vin;
  m = mfdin;
  VC = (ScramAttitude_VCore*) GC->P.findVC(v);
  return;
}
