// ==============================================================
//
//	ScramAttitude (Button Handling Code)
//	===============================
//
//	Copyright (C) 2016-2017	Andrew (ADSWNJ) Stokes
//                   All rights reserved
//
//	See ScramAttitude.cpp
//
// ==============================================================

#include "ScramAttitude.hpp"
#include <math.h>

// ==============================================================
// MFD button hooks to Button Page library
//
char* ScramAttitude::ButtonLabel (int bt)
{
	return LC->B.ButtonLabel(bt);
}

// Return button menus
int ScramAttitude::ButtonMenu (const MFDBUTTONMENU **menu) const
{
	return LC->B.ButtonMenu(menu);
}

// Return clicked button
bool ScramAttitude::ConsumeButton (int bt, int event) {
  return LC->B.ConsumeButton(this, bt, event);
}

// Return pressed keystroke
bool ScramAttitude::ConsumeKeyBuffered (DWORD key) {
  return LC->B.ConsumeKeyBuffered(this, key);
}



// ==============================================================
// MFD Button Handler Callbacks
//

// AP = AutoPilot On/Off
void ScramAttitude::Button_AP() {
  VC->apState = (VC->apState ? 0 : 1);
  VC->warpLock = false;
  if (VC->apState == 1) {
    VC->vAccPrevTgt = VC->vAccAvg;
  }
  return;
};

// DWN = Reduce Dyn Pressure Target
void ScramAttitude::Button_DN() {
  VC->DPTgt -= 1.0;
  return;
};

// UP = Increase Dyn Pressure Target
void ScramAttitude::Button_UP() {
  VC->DPTgt += 1.0;
  return;
};

// LOG = Toggle Log
void ScramAttitude::Button_LOG() {
  if (VC->logState == 2) {
    return;
  }
  if (VC->logState == 0) {
    VC->logOpen();
  } else {
    VC->logClose();
  }
  return;
};

// DIA = Toggle Diags
void ScramAttitude::Button_DIA() {
  VC->showDiags = !VC->showDiags;
  return;
};


// A2D = AP VACC --
void ScramAttitude::Button_A2D() {
  VC->apState = 2;
  VC->vAccTgt -= 1.0;
  VC->warpLock = false;
  return;
};

// A1D = AP VACC = -1
void ScramAttitude::Button_A1D() {
  VC->apState = 2;
  VC->vAccTgt = -1.0;
  VC->warpLock = false;
  return;
};

// A0 = AP VACC = 0
void ScramAttitude::Button_A0() {
  VC->apState = 2;
  VC->vAccTgt = 0.0;
  VC->warpLock = false;
  return;
};

// A1U = AP VACC = +1
void ScramAttitude::Button_A1U() {
  VC->apState = 2;
  VC->vAccTgt = 1.0;
  VC->warpLock = false;
  return;
};

// A1U = AP VACC ++
void ScramAttitude::Button_A2U() {
  VC->apState = 2;
  VC->vAccTgt += 1.0;
  VC->warpLock = false;
  return;
};

// APA = AP Auto-Control
void ScramAttitude::Button_APA() {
  VC->apState = 1;
  VC->vAccTgt = 0.0;
  VC->warpLock = false;
  return;
};

// REA = Reset Error Average
void ScramAttitude::Button_REA() {
  VC->sumAbsErr = 0.0;
  VC->startSumAbsErr = VC->lastSumAbsErr = oapiGetSimTime();
  return;
};


// NULL Button
void ScramAttitude::Button_NUL() {
  return;
};
