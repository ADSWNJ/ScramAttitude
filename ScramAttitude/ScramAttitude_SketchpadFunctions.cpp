// ==========================================================================
//
//	ScramAttitude (Sketchpad Helper Functions for MFD Display Update)
//	=================================================================
//
//	Copyright (C) 2016-2017	Andrew (ADSWNJ) Stokes
//                   All rights reserved
//
//  A collection of helper functions to simplify writing on the sketchpad
//	See ScramAttitude.cpp
//
// ===========================================================================

#include "ScramAttitude.hpp"
#include "DisplayEngUnitFunctions.h"
#include <math.h>
#include <stdarg.h>

// MFD Positioning Helper Functions
int ScramAttitude::_Line(const int row) {  // row is 0-24, for 24 rows. e.g. Line(12)
  int ret;
  ret = (int)((H - (int)(ch / 4)) * row / 25) + (int)(ch / 4);
  return ret;
};

int ScramAttitude::_Col(const int pos) {  // pos is 0-5, for 6 columns. Eg Col(3) for middle
  int ret = (int)((W - (int)(cw / 2)) * pos / 6) + int(cw / 2);
  return ret;
};

int ScramAttitude::_Col2(const int pos) {  // pos is 0-11, for 12 columns. Eg Col(6) for middle
  int ret = (int)((W - (int)(cw / 2)) * pos / 12) + int(cw / 2);
  return ret;
};


// MFD Format and print helper
void ScramAttitude::skpFormatText(const int col, const int line, const char* fmt, ...) {
  LC->skpColPix = _Col(col);
  LC->skpLinePix = _Line(line);
  va_list args;
  va_start(args, fmt);
  vsprintf_s(LC->skpBuf, 128, fmt, args);
  va_end(args);
  LC->skp->Text(LC->skpColPix, LC->skpLinePix, LC->skpBuf, strlen(LC->skpBuf));
}

void ScramAttitude::skpFmtColText(const int col, const int line, const bool test, const DWORD truecol, const DWORD falsecol, const char* fmt, ...) {
  LC->skp->SetTextColor(test ? truecol : falsecol);
  LC->skpColPix = _Col(col);
  LC->skpLinePix = _Line(line);
  va_list args;
  va_start(args, fmt);
  vsprintf_s(LC->skpBuf, 128, fmt, args);
  va_end(args);
  LC->skp->Text(LC->skpColPix, LC->skpLinePix, LC->skpBuf, strlen(LC->skpBuf));
  LC->skp->SetTextColor(falsecol);
}

void ScramAttitude::skpColor(const DWORD col) {
  LC->skp->SetTextColor(col);
}

void ScramAttitude::skpFmtEngText(const int col, const int line, const char* fmt, const char* sfx, const double val, const int dloB) {
  LC->skpColPix = _Col(col);
  LC->skpLinePix = _Line(line);
  char engUnit[12] = "pnum kMGTPE";
  double cnvVal = val;
  int i = 4;
  int loB = LC->skpLoB + dloB;

  if (loB<-4) loB = -4;
  if (loB>6) loB = 6;
  loB += 4;

  if (fabs(cnvVal) < 1) {
    while ((fabs(cnvVal) < 1) && (i>loB)) {
      i--;
      cnvVal *= 1000;
    }
  } else if (fabs(cnvVal) >= 1000) {
    while ((fabs(cnvVal) >= 1000) && (i< 10)) {
      i++;
      cnvVal /= 1000;
    }
  }
  while (i<loB) {
    i++;
    cnvVal /= 1000;
  }
  if (engUnit[i] == ' ') {
    sprintf_s(LC->skpFmtBuf, 128, "%s%s", fmt, sfx);
    sprintf_s(LC->skpBuf, 128, LC->skpFmtBuf, cnvVal);
  } else {
    sprintf_s(LC->skpFmtBuf, 128, "%s%%c%s", fmt, sfx);
    sprintf_s(LC->skpBuf, 128, LC->skpFmtBuf, cnvVal, engUnit[i]);
  }
  LC->skp->Text(LC->skpColPix, LC->skpLinePix, LC->skpBuf, strlen(LC->skpBuf));
}


void ScramAttitude::skpFmtEngText(const int col, const int line, const char* fmt, const unsigned char* sfx, const double val, const int dloB) {
  LC->skpColPix = _Col(col);
  LC->skpLinePix = _Line(line);
  char engUnit[12] = "pnum kMGTPE";
  double cnvVal = val;
  int i = 4;
  int loB = LC->skpLoB + dloB;

  if (loB<-4) loB = -4;
  if (loB>6) loB = 6;
  loB += 4;

  if (fabs(cnvVal) < 1) {
    while ((fabs(cnvVal) < 1) && (i>loB)) {
      i--;
      cnvVal *= 1000;
    }
  } else if (fabs(cnvVal) >= 1000) {
    while ((fabs(cnvVal) >= 1000) && (i< 10)) {
      i++;
      cnvVal /= 1000;
    }
  }
  while (i<loB) {
    i++;
    cnvVal /= 1000;
  }
  if (engUnit[i] == ' ') {
    sprintf_s(LC->skpFmtBuf, 128, "%s%s", fmt, sfx);
    sprintf_s(LC->skpBuf, 128, LC->skpFmtBuf, cnvVal);
  } else {
    sprintf_s(LC->skpFmtBuf, 128, "%s%%c%s", fmt, sfx);
    sprintf_s(LC->skpBuf, 128, LC->skpFmtBuf, cnvVal, engUnit[i]);
  }
  LC->skp->Text(LC->skpColPix, LC->skpLinePix, LC->skpBuf, strlen(LC->skpBuf));
}


void ScramAttitude::skpTitle(const char *title) {
  Title(LC->skp, title);
  LC->skp->SetTextAlign(oapi::Sketchpad::LEFT, oapi::Sketchpad::BOTTOM);
  //LC->skp->SetTextColor(WHITE);
}


bool ScramAttitude::DisplayMessageMode() {
  skpTitle("ScramAttitude MFD");
  showMessage();
  return true;
};


// MFD Line formatting helper
void ScramAttitude::showMessage() {

  char localMsg[750];
  strcpy_s(localMsg, 750, LC->Message.c_str());
  char *bp = localMsg;
  char *bp2 = localMsg;
  char *bp3 = localMsg;
  char c1, c2;
  int i = 0;
  int j;
  int l = 4;
  bool eol = false;

  do {
    if ((*bp2 == '\n') || (*bp2 == '\0')) {     // Look for user newline or end of buffer
      eol = true;
      c1 = *bp2;
      *bp2 = '\0';
    } else {
      if (i == 34) {                              // 34 chars no newline ... need to break the line
        eol = true;
        bp3 = bp2;
        for (j = 34; j>20; j--) {                 // look for a space from 21 to 34
          if (*bp3 == ' ') break;
          bp3--;
        }
        if (j>20) {                             // space found
          bp2 = bp3;
          c1 = *bp2;
          *bp2 = '\0';
        } else {                                // no space ... insert hyphen
          bp3 = bp2 + 1;
          c1 = *bp2;
          c2 = *bp3;
          *bp2 = '-';
          *bp3 = '\0';
        }
      } else {                                  // Scan forward      
        i++;
        bp2++;
      }
    }

    if (eol) {                                  // EOL flag ... write out buffer from bp to bp2.
      LC->skp->Text(_Col(0), _Line(l++), bp, strlen(bp));
      eol = false;
      if (c1 == '\0') {
        bp = bp2;     // End of buffer
      } else if ((c1 == '\n') || (c1 == ' ')) {
        bp = bp2 + 1;   // Reset for next line of the buffer
        bp2++;
        i = 0;
      } else {
        bp = bp2;     // Put back the chars we stomped
        *bp2 = c1;
        *bp3 = c2;
        i = 0;
      }
    }
  } while (*bp);

  return;
}

