// ==============================================================
//
//	ScramAttitude (Vessel Core Header)
//	=============================
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
#include <list>
#include <string>
#include <vector>
using namespace std;

#ifndef _SCRAM_ATT_VCORE_H
#define _SCRAM_ATT_VCORE_H

//+++++
// Vessel Persistence core. One of these is instantiated per Vessel flown with this MFD up.
//+++++

class ScramAttitude_VCore {
  public:
    void corePreStep(double p_simT,double p_simDT,double p_mjd);

    // Core references ... instantiation, vessel reference and GC.
    ScramAttitude_VCore(VESSEL *vin, ScramAttitude_GCore* gcin);
    ~ScramAttitude_VCore();
    ScramAttitude_GCore* GC;


		// Add Vessel data here
    VESSEL *v;
    int vix; // Vessel index in LU
    char clName[128];

    // Logging Control
    void dumpMW();
    void logOpen();
    void logWrite();
    void logClose();
    void logError(errno_t err, const char *fmt...);
    void logError(const char *fmt...);

    // Calculate position, velocity, acceleration trends. Always make MAX_HIST odd, to have a defined middle value    
#define MAX_HIST 127
    double simT_hist[MAX_HIST];                  
    double alt_hist[MAX_HIST];
    double DP_hist[MAX_HIST];
    int histIx{ 0 };
    int histOld{ 1 };
    int histMid{ (MAX_HIST + 1) / 2 };

    int reqHist{ MAX_HIST };
    double minSimD{ 0.1 };

    // Averages from history calculations
    double altAvg;
    double vSpdAvg;
    double vAccAvg;
    double DPSpdAvg;
    double DPAccAvg;

    double mjd;
    double simT; 
    double simDT;
    double lastSimT{ 0.0 };

    bool showDiags{ false };

    bool warpLock{ false };                      // Flags up a warning if we try to warp under full auto AP (it's too unstable)

    double apLastSimT{ -3.0 };                   // Last time we ran the AP

    int logState{ 0 };                           // If logging, create DUMP.CSV in the ScramAttitude folder
    int apState{ 0 };                            // 0 = off, 1 = on auto, 2 = on manual VACC control 
    FILE *hLogFile;                              // Logging file handle

    double DPTgt{ 10.0 };                       // Vessel Dynamic Pressure Target
    double trimTgt{ 0.0 };                      // Vessel Elevator Position Target
    double vAccTgt{ 0.0 };                      // Vessel Vertical Acceleration Target
    double vAccPrevTgt{ 0.0 };                 // last VACC target (for delta calcs)
    double vAccMW;                              // Vessel Vertical Acceleration Target from Macvicar-Whelan fuzzy logic function

    double TAS;                                  // Vessel True Air Speed
    double mach;                                 // Vessel Speed (Mach)
    double DP{ 0.0 };                            // Vessel Dynamic Pressure Actual (kPa)
    double DPDelta{ 0.0 };                      // Delta to the target DP, projected froward a few secs
    double trim{ 0.0 };                          // Vessel Elevator Position Actual
    double alt{ 0.0 };                           // Vessel Altitude (meters)

    double sumAbsErr{ 0.0 };                     // sum of DP error (for AP tuning)
    double lastSumAbsErr;                        // simT of last error reading
    double startSumAbsErr;                       // simT of start of err
    double maxMach;                              // holds the fastest speed seen, so if you drop off it will warn you are slowing
    

    double MW_v0{ 0.0 };                          // Macvicar-Whelan top left matrix value
    double MW_v1{ 0.0 };                          // Macvicar-Whelan top right matrix value
    double MW_v2{ 0.0 };                          // Macvicar-Whelan bot left matrix value
    double MW_v3{ 0.0 };                          // Macvicar-Whelan bot right matrix value
    double MW_w0{ 0.0 };                          // Macvicar-Whelan top left matrix weight
    double MW_w1{ 0.0 };                          // Macvicar-Whelan top right matrix weight
    double MW_w2{ 0.0 };                          // Macvicar-Whelan bot left matrix weight
    double MW_w3{ 0.0 };                          // Macvicar-Whelan bot right matrix weight

    int MW_E0{ 0 };                               //  Macvicar-Whelan error floor
    int MW_DE0{ 0 };                              //  Macvicar-Whelan delta error floor
    double MW_E0W{ 0.0 };                         //  Macvicar-Whelan error weight
    double MW_DE0W{ 0.0 };                        //  Macvicar-Whelan delta error weight
    double MW_RESP{ 0.0 };                        //  Macvicar-Whelan response

    int MW_diag{ 1 };                             // Display MW on MFD panel

    char MW_desc[10][3]{ "NX","NL","NM","NS","Z0","PS","PM","PL","PX", "PX" };   // Macvicar - Whelan errors (Negative X-Large, large, Mid, Small, Zero, Postive ...)
    struct start_end_struct {
      double start;
      double end;
    };
    
    struct start_end_struct MW_SegmentationTableE[8] = {// Macvicar-Whelan Ramp Control matrix for DP Error (offset to target DP) 
      { 15.0,  8.0 },   // NX to NL
      {  4.0,  1.2 },    // NL to NM 
      {  0.8,  0.5 },    // NM to NS
      {  0.5,  0.0 },    // NS to Z0
      {  0.0, -0.4 },    // Z0 to PS
      { -0.4, -0.7 },   // PS to PM
      { -1.0, -2.0 },   // PM to PL
      { -4.0, -5.0 }    // PL to PX 
    }; 

    struct start_end_struct MW_SegmentationTableDE[8]{  // Macvicar-Whelan Ramp Control matrix for DP Delta-Error (rate of change of DP error) 
      {  0.40,   0.05 },  // NX to NL
      {  0.05,   0.03 },  // NL to NM
      {  0.03,   0.012 },  // NM to NS
      {  0.012,  0.00 },  // NS to Z0
      {  0.00,  -0.012 },  // Z0 to PS
      { -0.012, -0.03 }, // PS to PM
      { -0.03,  -0.05 }, // PM to PL
      { -0.05,  -0.30 }  // PL to PX 
    };


    double MW_DesiredVAccTable[9]{                    // Macvicar-Whelan Output goal table, for Negative X, L, M, S demand, Zero demand, and Positive S, M, L, X
      12.00,             // NX
       5.00,             // NL
       3.00,             // NM
       1.50,             // NS
       0.00,             // Z0
      -1.50,             // PS
      -3.00,             // PM
      -5.00,             // PL
     -12.00              // PX
    };


    double trim_ControlTable[5][2]{               // Governs the trim adjustment
      {0.25, 0.10},                               // < 0.05 = 0.20%, 0.20 to 0.30 is 0.3%, etc
      {0.60, 0.20},                               
      {1.20, 0.40},
      {2.50, 1.00},
      {5.00, 4.00}
    };

#define MW_NX -4
#define MW_NL -3
#define MW_NM -2
#define MW_NS -1
#define MW_Z0 0
#define MW_PS 1
#define MW_PM 2
#define MW_PL 3
#define MW_PX 4

    // Input from the E and dE fuzz functions is -4 to 4. Select the 4 entries corresponsing to (E, dE), (E+1,dE), (E, dE+1), (E+1, dE+1)
    // (Hence this is an 10 by 10, to handle E or dE in range -4 to +5 ... i.e. sometimes E or dE is 4.0, so we need the final column duplicated - i.e. the tramline)
    int MW_ControlStateTable[10][10]{
      //E:-4    -3     -2     -1      0      1      2      3      4      Tramline
      //E:NX    NL     NM     NS     Z0     PS     PM     PL     PX                     dE: 
      { MW_NX, MW_NX, MW_NX, MW_NL, MW_NL, MW_NL, MW_NM, MW_NS, MW_PS,    MW_PS },   // NX  -4
      { MW_NX, MW_NL, MW_NL, MW_NL, MW_NL, MW_NM, MW_NS, MW_Z0, MW_PL,    MW_PL },   // NL  -3
      { MW_NX, MW_NL, MW_NL, MW_NL, MW_NM, MW_NS, MW_Z0, MW_PS, MW_PX,    MW_PX },   // NM  -2
      { MW_NX, MW_NL, MW_NL, MW_NM, MW_NS, MW_Z0, MW_PS, MW_PM, MW_PX,    MW_PX },   // NS  -1
      { MW_NX, MW_NL, MW_NM, MW_NS, MW_Z0, MW_PS, MW_PM, MW_PL, MW_PX,    MW_PX },   // Z0   0
      { MW_NX, MW_NM, MW_NS, MW_Z0, MW_PS, MW_PM, MW_PL, MW_PL, MW_PX,    MW_PX },   // PS  +1
      { MW_NX, MW_NS, MW_Z0, MW_PS, MW_PM, MW_PL, MW_PL, MW_PL, MW_PX,    MW_PX },   // PM  +2
      { MW_NL, MW_Z0, MW_PS, MW_PM, MW_PL, MW_PL, MW_PL, MW_PL, MW_PX,    MW_PX },   // PL  +3
      { MW_NS, MW_PS, MW_PM, MW_PL, MW_PL, MW_PL, MW_PX, MW_PX, MW_PX,    MW_PX },   // PX  +4

      { MW_NS, MW_PS, MW_PM, MW_PL, MW_PL, MW_PL, MW_PX, MW_PX, MW_PX,    MW_PX }    //  Tramline
    };
    double MW_ControlTable[10][10];


  private:
    double Macvicar_Whelan_Err_Function(double E);
    double Macvicar_Whelan_dErr_Function(double dE);
    double Macvicar_Whelan_Fuzzy_Function(double E, double dE);

    bool GetVesselClassControlSettings();

};


#endif // _SCRAM_ATT_VCORE_H




