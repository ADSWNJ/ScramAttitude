// ====================================================================================================================//
//	ScramAttitude MFD
//	=================
//
//	Copyright (C) 2016-2017	Andrew (ADSWNJ) Stokes
//                   All rights reserved
//
//	Description:
//
//	This is a simple attitude control MFD to trim elevators on SCRAM-equipped vessels to maintain an optimal Dynamic Pressure 
//
//	Copyright Notice: 
//
//	This program is free software: you can redistribute it and/or modify
//	it under the terms of the GNU General Public License as published by
//	the Free Software Foundation, either version 3 of the License, or
//	(at your option) any later version.
//
//	This program is distributed in the hope that it will be useful,
//	but WITHOUT ANY WARRANTY; without even the implied warranty of
//	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//	GNU General Public License for more details.
//
//	For full licencing terms, pleaserefer to the GNU General Public License
//	(gpl-3_0.txt) distributed with this release, or see
//	http://www.gnu.org/licenses/.
//
//
//	Credits:
//	
//	Orbiter Simulator	(c) 2003-2016 Martin (Martins) Schweiger
// 	MFDButtonPage		(c) 2012-2016 Szymon (Enjo) Ender
//	
//
//	Release History:
//
//  V1.00	Initial Release
// ====================================================================================================================

#define STRICT
#define ORBITER_MODULE

#include "windows.h"
#include "orbitersdk.h"
#include "ScramAttitude.hpp"
#include "ScramAttitude_GCore.hpp"
#include "ScramAttitude_VCore.hpp"
#include "ScramAttitude_LCore.hpp"
#include "ParseFunctions.h"
#include "MFDPersist.hpp"

// ====================================================================================================================
// Global variables

ScramAttitude_GCore *g_SC = nullptr;    // points to the static persistence core

// ====================================================================================================================
// MFD class implementation

// Constructor executes on any F8, any resize of an ExtMFD, or any vessel switch
ScramAttitude::ScramAttitude (DWORD w, DWORD h, VESSEL *vessel, UINT mfd)
: MFD2 (w, h, vessel)
{
  if (g_SC == nullptr) {
    g_SC = new ScramAttitude_GCore();
    GC = g_SC;
  }
  GC = g_SC;


  VC = (ScramAttitude_VCore*) GC->P.findVC(vessel);		  // Locate our vessel core
  if (!VC) {
    VC = new ScramAttitude_VCore(vessel, GC);				    // ... if missing, initialize it.
    GC->P.addVC(vessel, VC);
  }

  LC = (ScramAttitude_LCore*) GC->P.findLC(vessel, mfd);	// Locate our local (vessl+MFD position) core
  if (!LC) {
    LC = new ScramAttitude_LCore(vessel, mfd, GC);			  // ... if missing, initialize it.
    GC->P.addLC(vessel, mfd, LC);
  }

  // Any construction for the display side of this MFD instance
  font = oapiCreateFont (h/25, true, "Fixed", FONT_NORMAL, 0);

  return;
}

ScramAttitude::~ScramAttitude ()
{
  oapiReleaseFont(font);
  //for (int i = 0; i < 12; i++) oapiReleasePen(pen[i]);
  return;
}





// ====================================================================================================================
// Save/load from .scn functions
void ScramAttitude::ReadStatus(FILEHANDLE scn) {

  char *line;
  char *ll;
  char *key;
  int pI;
  double pD;
  double pDA[16];
  bool outTableValChanged{ false };

  while (oapiReadScenario_nextline(scn, line)) {

    ll = line;
    if (!ParseString(&ll, &key)) break;
    if (!_stricmp(key, "END_MFD")) break;

    if (!_stricmp(key, "AP_MODE")) {
      if (!ParseInt(&ll, &pI)) continue;
      if (pI == 0 || pI == 1 || pI == 2) VC->apState = pI;
      continue;
    }

    if (!_stricmp(key, "LOG_MODE")) {
      if (!ParseInt(&ll, &pI)) continue;
      if (pI == 0) {
        if (VC->logState == 1) {
          VC->logClose();
        }
      } else if (pI == 1) {
        if (VC->logState == 0) {
          VC->logOpen();
        }
      }
      continue;
    }

    if (!_stricmp(key, "DIAG_MODE")) {
      if (!ParseInt(&ll, &pI)) continue;
      if (pI == 0) VC->showDiags = false;
      else if (pI == 1) VC->showDiags = true;
      continue;
    }

    if (!_stricmp(key, "DP_TGT")) {
      if (!ParseDouble(&ll, &pD)) continue;
      if (pD < 1.0) continue;
      if (pD > 200.0) continue;
      VC->DPTgt = pD;
      continue;
    }

    if (!_stricmp(key, "VACC_TGT")) {
      if (!ParseDouble(&ll, &pD)) continue;
      if (pD < -30.0) continue;
      if (pD > 30.0) continue;
      VC->vAccTgt = pD;
      continue;
    }

    if (!_stricmp(key, "TRIM_CTL")) {
      if (!ParseDoubleArray(&ll, pDA, 10)) continue;
      bool valid = true;
      for (int i = 0; i < 10; i++) {
        if (pDA[i] < 0.0 || pDA[i] > 10.0) valid = false;
      }
      if (!valid) continue;
      for (int i = 0, j=0; i < 5; i++) {
        VC->trim_ControlTable[i][0] = pDA[j++];
        VC->trim_ControlTable[i][1] = pDA[j++];
      }
      continue;
    }

    if (!_stricmp(key, "SEG_DE_CTL")) {
      if (!ParseDoubleArray(&ll, pDA, 16)) continue;
      bool valid = true;
      for (int i = 0; i < 16; i++) {
        if (pDA[i] < -10.0 || pDA[i] > 10.0) valid = false;
      }
      if (!valid) continue;
      for (int i = 0, j = 0; i < 8; i++) {
        VC->MW_SegmentationTableDE[i].start = pDA[j++];
        VC->MW_SegmentationTableDE[i].end = pDA[j++];
      }
      continue;
    }

    if (!_stricmp(key, "SEG_E_CTL")) {
      if (!ParseDoubleArray(&ll, pDA, 16)) continue;
      bool valid = true;
      for (int i = 0; i < 16; i++) {
        if (pDA[i] < -50.0 || pDA[i] > 50.0) valid = false;
      }
      if (!valid) continue;
      for (int i = 0, j = 0; i < 8; i++) {
        VC->MW_SegmentationTableE[i].start = pDA[j++];
        VC->MW_SegmentationTableE[i].end = pDA[j++];
      }
      continue;
    }

    if (!_stricmp(key, "DES_VACC_CTL")) {
      if (!ParseDoubleArray(&ll, pDA, 9)) continue;
      bool valid = true;
      for (int i = 0; i < 9; i++) {
        if (pDA[i] < -50.0 || pDA[i] > 50.0) valid = false;
      }
      if (!valid) continue;
      outTableValChanged = true;
      for (int i = 0; i < 9; i++) {
        VC->MW_DesiredVAccTable[i] = pDA[i];
      }
      continue;
    }

  }
  if (outTableValChanged) {
    // Reassign the fuzzy matrix after load
    for (int r = 0; r < 10; r++) {
      for (int c = 0; c < 10; c++) {
        VC->MW_ControlTable[r][c] = VC->MW_DesiredVAccTable[VC->MW_ControlStateTable[r][c] + 4];
      }
    }
  }
  return;
  
}

void ScramAttitude::WriteStatus(FILEHANDLE scn) const {
  char buf[256];

  oapiWriteScenario_float(scn, "DP_TGT", VC->DPTgt);
  oapiWriteScenario_float(scn, "VACC_TGT", VC->vAccTgt);
  oapiWriteScenario_int(scn, "AP_MODE", VC->apState);
  oapiWriteScenario_int(scn, "LOG_MODE", VC->logState);
  oapiWriteScenario_int(scn, "DIAG_MODE", VC->showDiags? 1 : 0);


  sprintf_s(buf, 128, "%.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f",
    VC->MW_SegmentationTableE[0].start,
    VC->MW_SegmentationTableE[0].end,
    VC->MW_SegmentationTableE[1].start,
    VC->MW_SegmentationTableE[1].end,
    VC->MW_SegmentationTableE[2].start,
    VC->MW_SegmentationTableE[2].end,
    VC->MW_SegmentationTableE[3].start,
    VC->MW_SegmentationTableE[3].end,
    VC->MW_SegmentationTableE[4].start,
    VC->MW_SegmentationTableE[4].end,
    VC->MW_SegmentationTableE[5].start,
    VC->MW_SegmentationTableE[5].end,
    VC->MW_SegmentationTableE[6].start,
    VC->MW_SegmentationTableE[6].end,
    VC->MW_SegmentationTableE[7].start,
    VC->MW_SegmentationTableE[7].end
  );
  oapiWriteScenario_string(scn, "SEG_E_CTL", buf);

  sprintf_s(buf, 128, "%.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f",
    VC->MW_SegmentationTableDE[0].start,
    VC->MW_SegmentationTableDE[0].end,
    VC->MW_SegmentationTableDE[1].start,
    VC->MW_SegmentationTableDE[1].end,
    VC->MW_SegmentationTableDE[2].start,
    VC->MW_SegmentationTableDE[2].end,
    VC->MW_SegmentationTableDE[3].start,
    VC->MW_SegmentationTableDE[3].end,
    VC->MW_SegmentationTableDE[4].start,
    VC->MW_SegmentationTableDE[4].end,
    VC->MW_SegmentationTableDE[5].start,
    VC->MW_SegmentationTableDE[5].end,
    VC->MW_SegmentationTableDE[6].start,
    VC->MW_SegmentationTableDE[6].end,
    VC->MW_SegmentationTableDE[7].start,
    VC->MW_SegmentationTableDE[7].end
  );
  oapiWriteScenario_string(scn, "SEG_DE_CTL", buf);

  sprintf_s(buf, 128, "%.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f",
    VC->MW_DesiredVAccTable[0],
    VC->MW_DesiredVAccTable[1],
    VC->MW_DesiredVAccTable[2],
    VC->MW_DesiredVAccTable[3],
    VC->MW_DesiredVAccTable[4],
    VC->MW_DesiredVAccTable[5],
    VC->MW_DesiredVAccTable[6],
    VC->MW_DesiredVAccTable[7],
    VC->MW_DesiredVAccTable[8]
  );
  oapiWriteScenario_string(scn, "DES_VACC_CTL", buf);

  sprintf_s(buf, 128, "%.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f",
    VC->trim_ControlTable[0][0],
    VC->trim_ControlTable[0][1],
    VC->trim_ControlTable[1][0],
    VC->trim_ControlTable[1][1],
    VC->trim_ControlTable[2][0],
    VC->trim_ControlTable[2][1],
    VC->trim_ControlTable[3][0],
    VC->trim_ControlTable[3][1],
    VC->trim_ControlTable[4][0],
    VC->trim_ControlTable[4][1]
  );
  oapiWriteScenario_string(scn, "TRIM_CTL", buf);

  return;
}
