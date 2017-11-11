// ==============================================================
//
//	ScramAttitude (Button Handling Headers)
//	==================================
//
//	Copyright (C) 2016-2017	Andrew (ADSWNJ) Stokes
//                   All rights reserved
//
//	See ScramAttitude.cpp
//
// ==============================================================



#ifndef _SCRAM_ATT_BUTTON_CLASS
#define _SCRAM_ATT_BUTTON_CLASS
#include "MFDButtonPage.hpp"

class ScramAttitude;

class ScramAttitude_Buttons : public MFDButtonPage<ScramAttitude>
{
  public:
    ScramAttitude_Buttons();
  protected:
    bool SearchForKeysInOtherPages() const;
  private:
};
#endif // _SCRAM_ATT_BUTTON_CLASS

