// ==============================================================
//
//	ScramAttitude MFD Headers
//	====================
//
//	Copyright (C) 2016-2017	Andrew (ADSWNJ) Stokes
//                   All rights reserved
//
//	See ScramAttitude.cpp
//
// ==============================================================


#ifndef __SCRAM_ATT_H
#define __SCRAM_ATT_H

#include "ScramAttitude_GCore.hpp"   
#include "ScramAttitude_VCore.hpp" 
#include "ScramAttitude_LCore.hpp" 


extern "C" 
class ScramAttitude: public MFD2
{
public:
	ScramAttitude (DWORD w, DWORD h, VESSEL *vessel, UINT mfd);
	~ScramAttitude ();

  char *ButtonLabel (int bt);
	int ButtonMenu (const MFDBUTTONMENU **menu) const;
  bool ConsumeKeyBuffered (DWORD key);
  bool ConsumeButton (int bt, int event);
  
  bool Update (oapi::Sketchpad *skp);
  bool DisplayMessageMode();

  // Button Press Handlers
  void Button_AP();
  void Button_DN();
  void Button_UP();
  void Button_LOG();
  void Button_A0();
  void Button_A1U();
  void Button_A1D();
  void Button_A2U();
  void Button_A2D();
  void Button_APA();
  void Button_REA();
  void Button_NUL();
  void Button_DIA();

  // Persistence functions
  void ReadStatus(FILEHANDLE scn);
  void WriteStatus(FILEHANDLE scn) const;

protected:
  ScramAttitude_GCore* GC;
  ScramAttitude_LCore* LC;
  ScramAttitude_VCore* VC;

  oapi::IVECTOR2 iv[10000];

  int _Line(const int row );
  int _Col(const int pos );
  int _Col2(const int pos );
  void skpFormatText(const int col, const int line, const char* fmt, ...);
  void skpFmtColText(const int col, const int line, const bool test, const DWORD truecol, const DWORD falsecol, const char* fmt, ...);
  void skpFmtEngText(const int col, const int line, const char* fmt, const char* sfx, const double val, const int dloB = 0);
  void skpFmtEngText(const int col, const int line, const char* fmt, const unsigned char* sfx, const double val, const int dloB = 0);
  void skpTitle(const char* title);
  void skpColor(DWORD col);
  void showMessage();

  //                  R     G           B
  DWORD CLR_WHITE  = 255 + 255 * 256 + 255 * 256 * 256;
  DWORD CLR_RED    = 238 +  32 * 256 +  77 * 256 * 256;
  DWORD CLR_YELLOW = 255 + 255 * 256 +   0 * 256 * 256;

  oapi::Font *font;

};

#endif // !__SCRAM_ATT_H