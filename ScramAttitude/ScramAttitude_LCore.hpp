// ==============================================================
//
//	ScramAttitude (Local Core Header)
//	============================
//
//	Copyright (C) 2016-2017	Andrew (ADSWNJ) Stokes
//                   All rights reserved
//
//	See ScramAttitude.cpp
//
// ==============================================================

#include "windows.h"
#include "orbitersdk.h"
#include "ScramAttitude_Buttons.hpp"
#include "MFDPersist.hpp"
#include "ScramAttitude_GCore.hpp"
#include "ScramAttitude_VCore.hpp"
#include <list>
#include <string>

using namespace std;

#ifndef _SCRAM_ATT_LCORE_H
#define _SCRAM_ATT_LCORE_H

//+++++
// Local Persistence core. One of these is instantiated per Vessel AND MFD panel location. Local defaults for that combination.
//+++++

class ScramAttitude_LCore {
  public:
    // Local references ... instantiation, references for vesseland mfd position, and links to the appropriate VC, MC and GC
    ScramAttitude_LCore(VESSEL *vin, UINT mfdin, ScramAttitude_GCore* gcin);
    VESSEL *v;
    UINT m;
    ScramAttitude_GCore* GC;
    ScramAttitude_VCore* VC;

    // Add local vessel+panel data here

    ScramAttitude_Buttons B;
    bool showMessage{ false };
    bool okMessagePage{ true };
    string Message;
    int mode{ 0 };
    int PrvNxtMode;

    oapi::Sketchpad *skp; // points to local sketchpad for this MFD and vessel
    int skpLoB;           // Lowest precision for skp eng numnber formatting
    char skpBuf[128];     // Formatting buffer for MFD updates
    char skpFmtBuf[128];  // Formatting buffer for MFD updates
    int skpColPix;        // X-offset pixel (top left origin)
    int skpLinePix;       // Y offsel pixel (top left origin)

};


#endif // _SCRAM_ATT_CORE_CLASSES