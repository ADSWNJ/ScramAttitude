// ==============================================================
//
//	ScramAttitude (MFD Display Update)
//	==================================
//
//	Copyright (C) 2016-2017	Andrew (ADSWNJ) Stokes
//                   All rights reserved
//
//	See ScramAttitude.cpp
//
// ==============================================================

#include "ScramAttitude.hpp"
#include "DisplayEngUnitFunctions.h"
#include <math.h>
#include <stdarg.h>


bool ScramAttitude::Update(oapi::Sketchpad *skp)
{
  LC->skp = skp;
  if (LC->showMessage) return DisplayMessageMode();

  skpTitle("ScramAttitude MFD");
  int l = 3;

  skpFormatText(2, l, "   Act");
  skpFormatText(4, l, "   Tgt");
  l++;
  skpFormatText(0, l, "DP:");
  skpFormatText(2, l, "%7.2f", VC->DP);
  skpFormatText(4, l, "%7.2f", VC->DPTgt);
  l++;
  skpFormatText(0, l, "VAcc:");
  skpFormatText(2, l, "%+7.2f", VC->vAccAvg);
  if (VC->apState > 0) skpFormatText(4, l, "%+7.2f", VC->vAccTgt);
  l++;
  skpFormatText(0, l, "Elev:");
  skpFormatText(2, l, "%+7.2f", VC->trim);
  if (VC->apState > 0) {
    if (VC->trimTgt == -1.0 || VC->trimTgt == 1.0) {
      skpColor(CLR_YELLOW);
      skpFormatText(4, l, "%+7.2f ***", VC->trimTgt);
      skpColor(CLR_WHITE);
    } else {
      skpFormatText(4, l, "%+7.2f", VC->trimTgt);
    }
  }
  l++;
  l++;

  skpFormatText(0, l, "Mach:");
  skpFormatText(2, l, "%7.2f", VC->mach);
  l++;
  skpFormatText(0, l, "VSpd:");
  skpFormatText(2, l, "%+7.2f", VC->vSpdAvg);
  l++;
  skpFormatText(0, l, "DPSpd:");
  skpFormatText(2, l, "%+7.2f", VC->DPSpdAvg);
  l++;

  skpFormatText(0, l, "DPErA:");
  if (VC->lastSumAbsErr > VC->startSumAbsErr) {
    skpFormatText(2, l, "%7.2f", VC->sumAbsErr / (VC->lastSumAbsErr - VC->startSumAbsErr));
  }

  l++;
  l++;

  skpFormatText(0, l, "AP:");
  switch (VC->apState) {
  case 0:
    skpFormatText(2, l, "OFF");
    break;
  case 1:
    skpFormatText(2, l, "ON AUTO VACC");
    break;
  case 2:
    skpFormatText(2, l, "ON FIXED VACC");
    break;
  }
  l++;
  skpFormatText(0, l, "Log:");
  switch (VC->logState) {
  case 0:
    skpFormatText(2, l, "OFF");
    break;
  case 1:
    skpFormatText(2, l, "ON");
    break;
  case 2:
    skpColor(CLR_YELLOW);
    skpFormatText(2, l, "FAILED TO OPEN");
    skpColor(CLR_WHITE);
    break;
  }
  l++;

  if (VC->MW_diag == 1 && VC->apState == 1 && VC->showDiags) {
    l++;
    // Display MW matrix and output
    skpFormatText(0, l, "DE\\E");
    skpFormatText(1, l, " %s", VC->MW_desc[VC->MW_E0]);
    skpFormatText(2, l, " %s", VC->MW_desc[VC->MW_E0+1]);
    l++;
    skpFormatText(0, l, "%s", VC->MW_desc[VC->MW_DE0]);
    skpFormatText(1, l, "%+4.1f", VC->MW_v0);
    skpFormatText(2, l, "%+4.1f", VC->MW_v2);
    skpFormatText(3, l, "%3.0f%%", VC->MW_w0 * 100.0);
    skpFormatText(4, l, "%3.0f%%", VC->MW_w2 * 100.0);
    l++;
    skpFormatText(0, l, "%s", VC->MW_desc[VC->MW_DE0+1]);
    skpFormatText(1, l, "%+4.1f", VC->MW_v1);
    skpFormatText(2, l, "%+4.1f", VC->MW_v3);
    skpFormatText(3, l, "%3.0f%%", VC->MW_w1 * 100.0);
    skpFormatText(4, l, "%3.0f%%", VC->MW_w3 * 100.0);
    l++;
    skpFormatText(0, l, "Out:");
    skpFormatText(1, l, "   %+.2f", VC->MW_RESP);
  }

  if (l<22) l = 22;
  // Warning area

  if (VC->apState > 0 && VC->mach < VC->maxMach * 0.98) {
    skpColor(CLR_RED);
    skpFormatText(0, l++, "Mach: CHECK SLOW SPEED");
  }
  if (VC->apState > 0 && VC->trimTgt == 1.0) {
    skpColor(CLR_YELLOW);
    skpFormatText(0, l++, "Elevator Control: AT MAX");
  }
  if (VC->apState > 0 && VC->trimTgt == -1.0) {
    skpColor(CLR_YELLOW);
    skpFormatText(0, l++, "Elevator Control: AT MIN");
  }
  if (VC->warpLock) {
    skpColor(CLR_YELLOW);
    skpFormatText(0, l++, "Warp: MAX 1.0 on AUTO VACC AP");
  }
  skpColor(CLR_WHITE);
  return true;

};