// ==============================================================
//
//	ScramAttitude (Vessel Core Persistence)
//	=======================================
//
//	Copyright (C) 2016-2017	Andrew (ADSWNJ) Stokes
//                   All rights reserved
//
//	See ScramAttitude.cpp
//
// ==============================================================

#include "ScramAttitude_GCore.hpp"
#include "ScramAttitude_VCore.hpp"
#include "ParseFunctions.h"
#include <stdio.h>
#include <math.h>
#include <string.h>
#include <assert.h>

ScramAttitude_VCore::ScramAttitude_VCore(VESSEL *vin, ScramAttitude_GCore* gcin) {
	// Vessel core constructor
  GC = gcin;
	v = vin;
  for (int i = 0; i < MAX_HIST; i++) {
    simT_hist[i] = 0.0;
    alt_hist[i] = v->GetAltitude();
    DP_hist[i] = v->GetDynPressure() / 1000.0;
  }
  maxMach = mach = v->GetMachNumber();
  startSumAbsErr = lastSumAbsErr = oapiGetSimTime();
  //dumpMW();

  for (int r = 0; r < 10; r++) {
    for (int c = 0; c < 10; c++) {
      MW_ControlTable[r][c] = MW_DesiredVAccTable[MW_ControlStateTable[r][c] + 4];
    }
  }

  return;
};

ScramAttitude_VCore::~ScramAttitude_VCore() {

}


void ScramAttitude_VCore::corePreStep(double p_simT,double p_simDT,double p_mjd) {

  simT = p_simT;
  simDT = p_simDT;
  mjd = p_mjd;

  TAS = v->GetAirspeed();
  mach = v->GetMachNumber();
  if (maxMach < mach) maxMach = mach;
  DP = v->GetDynPressure() / 1000.0;

  if (lastSumAbsErr + 1.0 < simT) {
    sumAbsErr += abs(DP - DPTgt);
    lastSumAbsErr = simT;
  }

  trim = v->GetControlSurfaceLevel(AIRCTRL_ELEVATORTRIM);
  alt = v->GetAltitude();

  alt_hist[histIx] = alt;
  DP_hist[histIx] = DP;
  simT_hist[histIx] = simT;

  if (simT_hist[MAX_HIST - 1] == 0.0) {
    histIx = (histIx + 1) % MAX_HIST;
    histOld = (histOld + 1) % MAX_HIST;
    histMid = (histMid + 1) % MAX_HIST;
    return; // Want to populate the history buffer before doing any control activity
  }

  altAvg = 0.0;
  vSpdAvg = 0.0;
  vAccAvg = 0.0;
  DPSpdAvg = 0.0;
  DPAccAvg = 0.0;

  for (int i = 0; i < MAX_HIST; i++) {
    altAvg += alt_hist[i];
  }
  altAvg /= (double) MAX_HIST;
   
  double vSpdH1 = (alt_hist[histMid] - alt_hist[histOld]) / (simT_hist[histMid] - simT_hist[histOld]);
  double vSpdH2 = (alt_hist[histIx] - alt_hist[histMid]) / (simT_hist[histIx] - simT_hist[histMid]);
  double DPSpdH1 = (DP_hist[histMid] - DP_hist[histOld]) / (simT_hist[histMid] - simT_hist[histOld]);
  double DPSpdH2 = (DP_hist[histIx] - DP_hist[histMid]) / (simT_hist[histIx] - simT_hist[histMid]);
  double simTH1 = (simT_hist[histMid] + simT_hist[histOld]) / 2.0;
  double simTH2 = (simT_hist[histIx] + simT_hist[histMid]) / 2.0;
  double a = oapiGetTimeAcceleration();
  if (a < 1.0) a = 1.0;

  vAccAvg = (vSpdH2 - vSpdH1) / (simTH2 - simTH1);
  vSpdAvg = (alt_hist[histIx] - alt_hist[histOld]) / (simT_hist[histIx] - simT_hist[histOld]);
  DPAccAvg = (DPSpdH2 - DPSpdH1) / (simTH2 - simTH1);
  DPSpdAvg = (DP_hist[histIx] - DP_hist[histOld]) / (simT_hist[histIx] - simT_hist[histOld]);

  histIx = (histIx + 1) % MAX_HIST;
  histOld = (histOld + 1) % MAX_HIST;
  histMid = (histMid + 1) % MAX_HIST;

#define fwd_s (2.0 * a)
  DPDelta = DP + DPSpdAvg * fwd_s + 0.5 * DPAccAvg * fwd_s * fwd_s - DPTgt;  /// Predict DP delta in 'fwd' secs
  vAccMW = Macvicar_Whelan_Fuzzy_Function(DPDelta, DPSpdAvg); // drive VSpd from the main fuzzy logic control function

  if (apState == 1) {
    vAccTgt = vAccMW;
    if (a > 1.0) {
      warpLock = true;
      oapiSetTimeAcceleration(1.0);
    }

 //   if (vAccTgt > vAccPrevTgt + 3.0 * simDT) {
 //     vAccTgt = vAccPrevTgt + 3.0 * simDT
 //   } else if (vAccTgt < vAccPrevTgt - 3.0 * simDT) {
 //       vAccTgt = vAccPrevTgt - 3.0 * simDT;
 //   }
  }

  if (apState > 0) {
    double trim_delta = 0.000;
    double absVaccErr = abs(vAccTgt - vAccAvg);
    for (int i = 0; i < 5; i++) {
      if (absVaccErr > trim_ControlTable[i][0]) {
        trim_delta = trim_ControlTable[i][1] * simDT / (100.0);
      } else {
        break;
      }
    }

    if (vAccTgt < vAccAvg) trimTgt -= trim_delta;
    else trimTgt += trim_delta;
    if (trimTgt > 1.0) trimTgt = 1.0;
    else if (trimTgt < -1.0) trimTgt = -1.0;

    v->SetControlSurfaceLevel(AIRCTRL_ELEVATORTRIM, trimTgt);
  }

  if (logState == 1) logWrite();
  vAccPrevTgt = vAccTgt;

  return;
}

void ScramAttitude_VCore::dumpMW() {
  FILE *hFile = nullptr;
  errno_t err = fopen_s(&hFile, ".\\Config\\MFD\\ScramAttitude\\MW.csv", "w");
  if (err != 0 || hFile == nullptr) return;

  fprintf(hFile, "E, f(E)\n");

#define Ehi 22.0
#define Elo -10.0
#define Estep -.3
#define dEhi .7
#define dElo -.7
#define dEstep -.03

  for (double E = Ehi; E >= Elo; E += Estep) {
    fprintf(hFile, "%.2f, %.3f\n", E, Macvicar_Whelan_Err_Function(E));
  }
  fprintf(hFile, "\n\ndE, f(dE)\n");
  for (double dE = dEhi; dE >= dElo; dE += dEstep) {
    fprintf(hFile, "%.2f, %.3f\n", dE, Macvicar_Whelan_dErr_Function(dE));
  }

  fprintf(hFile, "\n\nCONTROL TABLE\n");
  fprintf(hFile, "E:, ");
  for (double E = Ehi; E >= Elo; E += Estep) {
    fprintf(hFile, "%.2f, ", E);
  }
  fprintf(hFile, "\ndE:\n");

  for (double dE = dEhi; dE >= dElo; dE += dEstep) {
    fprintf(hFile, "%.2f", dE);
    for (double E = Ehi; E >= Elo; E += Estep) {
      fprintf(hFile, ", %.2f", Macvicar_Whelan_Fuzzy_Function(E, dE));
    }
    fprintf(hFile, "\n");
  }
  fclose(hFile);
}

void ScramAttitude_VCore::logOpen() {
  logState = 0;
  errno_t err = fopen_s(&hLogFile, ".\\Config\\MFD\\ScramAttitude\\Dump.csv", "w");
  if (err != 0 || hLogFile == nullptr) {
    logError(err, "trying to open .\\Config\\MFD\\ScramAttitude\\Dump.csv at MJD %.5f", mjd);
    hLogFile = nullptr;
    return;
  }

  int bytesWritten;
  bytesWritten = fprintf(hLogFile, "MJD, T, DT, AP, Mach, TAS, Alt, VSpd, VAcc,    DP_tgt, DP_act, DP_spd, DP_acc,   Trim_act, Trim_tgt, VAcc_tgt, MW_Seg(E), MW_Seg(DE), MW_tl, MW_bl, MW_tr, MW_br, MW_tl%%, MW_bl%%, MW_tr%%, MW_br%%,  T, MW_Out, DP_d, VAcc, SumAbsErr\n");
  if (bytesWritten < 0) {
    logError("writing MJD %.5f", mjd);
    return;
  }
  logState = 1;
  return;
}

void ScramAttitude_VCore::logWrite() {
  int bytesWritten;
  double fE = Macvicar_Whelan_Err_Function(DPDelta);
  double fdE = Macvicar_Whelan_dErr_Function(DPSpdAvg);
  bytesWritten = fprintf(hLogFile, "%.5f, %.3f, %.5f,    %d,      %.3f, %.1f, %.1f, %.3f, %.3f,          %.1f, %.3f, %.3f, %.3f,          %.4f, %.4f, %.1f, %.2f, %0.2f, %.2f, %.2f,%.2f, %.2f, %.2f, %.2f, %.2f, %.2f,           %.3f, %.2f, %.3f, %.3f, %.3f\n",
                                     mjd, simT, simDT,   apState, mach, TAS, alt, vSpdAvg, vAccAvg,     DPTgt, DP, DPSpdAvg, DPAccAvg,   trim, trimTgt, vAccTgt, fE, fdE, MW_v0, MW_v1, MW_v2, MW_v3, MW_w0, MW_w1, MW_w2, MW_w3,      simT, vAccMW, DPDelta, vAccAvg, sumAbsErr);
  if (bytesWritten < 0) {
    logError("writing log at MJD %.5f", mjd);
  }
  return;
}

void ScramAttitude_VCore::logClose() {
  fclose(hLogFile);
  hLogFile = nullptr;
  logState = 0;
  return;
}

void ScramAttitude_VCore::logError(errno_t err, const char *fmt...) {
  char buf1[256];
  char fmtBuf[256];
  char buf2[256];
  
  sprintf_s(buf1, "   >>> ScramAltitude: Error %d (%s) ", err, strerror(err));
  sprintf_s(fmtBuf, 256, "%s%s", buf1, fmt);
  va_list args;
  va_start(args, fmt);
  vsprintf_s(buf2, 256, fmtBuf, args);
  va_end(args);
  oapiWriteLog(buf2);
  if (hLogFile) fclose(hLogFile);
  hLogFile = nullptr;
  logState = 2;
  return;
}

void ScramAttitude_VCore::logError(const char *fmt...) {
  char buf1[256];
  char fmtBuf[256];
  char buf2[256];

  sprintf_s(buf1, "   >>> ScramAltitude: Error ");
  sprintf_s(fmtBuf, 256, "%s%s", buf1, fmt);
  va_list args;
  va_start(args, fmt);
  vsprintf_s(buf2, 256, fmtBuf, args);
  va_end(args);
  oapiWriteLog(buf2);
  if (hLogFile) fclose(hLogFile);
  hLogFile = nullptr;
  logState = 2;
  return;
}

// My Macvicar-Whelan control matrix is based on work in this 1994 paper:
// "A Fuzzy Supervisor for PD Control of Unknown Systems" Robert P. Copeland, Kuldip S. Rattan of Wright State University
// http://cecs.wright.edu/~krattan/courses/419/louisville.pdf  <<-- nice work, guys, driving a space simulation control loop 23 years later!
//
double ScramAttitude_VCore::Macvicar_Whelan_Err_Function(double E) {
  // Generate a double representing the trapezoid fuzzing function from the error, for the Macvicar-Whelan fuzz control
  // Response is in the range -4.0 to 4.0
  // -4.00 represents Negative X-Large (NX) response
  // -3.00 represents Negative Large (NL) response
  // -2.99 .. -2.01 represents the blend of NL and NM
  // -2.00 represents the Negative Medium (NM) response
  // -1.99 .. -1.01 represents the blend of NM to NS
  // -1.00 represents the Negative Small (NS) response
  // -1.00 .. -0.01 rpresents the blend of NS to Zero
  // 0.00 represents the Zero (Z0) response
  // ... etc for positives: PS, PM, PL , PX

  // Input E represents the error in Dyn Pressure. E.g. want  10.0 kPa, current 30.0 kPa => E = 20.0  



  double fuzz = -4.0;

  for (int i = 0; i < 8; i++) {
    if (E >= MW_SegmentationTableE[i].start) break;                                 // pre-ramp... done, and return the whole number
    if (E >= MW_SegmentationTableE[i].end) {
      fuzz += (MW_SegmentationTableE[i].start - E) / (MW_SegmentationTableE[i].start - MW_SegmentationTableE[i].end); // mid-ramp ... done, and return the fraction along ramp
      break;
    } 
    fuzz += 1.0;                                                   // post-ramp ... go to the next test
  }
  return fuzz;
}
double ScramAttitude_VCore::Macvicar_Whelan_dErr_Function(double dE) {
  // Generate a double representing the trapezoid fuzzing function from the delta-error, for the Macvicar-Whelan fuzz control
  // Response is in the range -4.0 to 4.0 as the Err function
  // Input dE represents the rate of change of Dyn Pressure. Goal to keep it in the range +-0.5. 

  double fuzz = -4.0;
  for (int i = 0; i < 8; i++) {
    if (dE >= MW_SegmentationTableDE[i].start) break;                                 // pre-ramp... done, and return the whole number
    if (dE >= MW_SegmentationTableDE[i].end) {
      fuzz += (MW_SegmentationTableDE[i].start - dE) / (MW_SegmentationTableDE[i].start - MW_SegmentationTableDE[i].end); // mid-ramp ... done, and return the fraction along ramp
      break;
    }
    fuzz += 1.0;                                                    // post-ramp ... go to the next test
  }
  return fuzz;
}

double ScramAttitude_VCore::Macvicar_Whelan_Fuzzy_Function(double E, double dE) {
  // Generate a desired control response from the Macvicar-Whelan fuzzy rule matrix
  // Search for "A Fuzzy Supervisor for PD Control of Unknown Systems" by Robert P. Copeland and Kuldip S. Rattan
  // For us, the control output is desired vertical speed (dVAS), which drives the elevator trim
  // Desired rate for this is represented by these NX..Z0..PX parameters

  double fE = Macvicar_Whelan_Err_Function(E);
  double fdE = Macvicar_Whelan_dErr_Function(dE);


  // find the top left of the 4 entries (i.e. lower index numbers). Fuzz comes in with -4 to +4, we need 0-8, so add 4
  // suppress the C4244 double to int possible loss of data ... we know what we are doing!
#pragma warning(push)
#pragma warning(disable:4244)
  int x = 4.0 + (int)floor(fdE);
  int y = 4.0 + (int)floor(fE);
#pragma warning(pop)

  // find the fractions of how close we are to the lower or upper index for each range
  double xf1 = fdE - floor(fdE);
  double yf1 = fE - floor(fE);
  double xf0 = 1.0 - xf1;
  double yf0 = 1.0 - yf1;

  // pull the 4 values
  MW_v0 = MW_ControlTable[x][y];
  MW_v1 = MW_ControlTable[x+1][y];
  MW_v2 = MW_ControlTable[x][y+1];
  MW_v3 = MW_ControlTable[x+1][y+1];

  // pull the 4 weights
  MW_w0 = xf0 * yf0;
  MW_w1 = xf1 * yf0;
  MW_w2 = xf0 * yf1;
  MW_w3 = xf1 * yf1;

  MW_E0 = y;
  MW_DE0 = x;
  MW_E0W = yf1;
  MW_DE0W = xf1;

  // do a weighted average of the values 
  MW_RESP = (MW_w0 * MW_v0 + MW_w1 * MW_v1 + MW_w2 * MW_v2 + MW_w3 * MW_v3) / (MW_w0 + MW_w1 + MW_w2 + MW_w3);

  return MW_RESP; 
}