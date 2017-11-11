// ==============================================================
//
//	ScramAttitude (MFD Button Management)
//	=====================================
//
//	Copyright (C) 2016-2017	Andrew (ADSWNJ) Stokes
//                   All rights reserved
//
//	See ScramAttitude.cpp
//
// ==============================================================

#include "MFDButtonPage.hpp"
#include "ScramAttitude_Buttons.hpp"
#include "ScramAttitude.hpp"


ScramAttitude_Buttons::ScramAttitude_Buttons() 
{
    static const MFDBUTTONMENU mnu0[] =
    {
      {"AP On/Off", 0, 'A'},
      {"Tgt DP Down", 0, 'D'},
      {"Tgt DP Up", 0, 'U'},
      {"Log On/Off", 0, 'L'},
      { "Reset Err Avg", 0, 'R' },
      { "Show Diags", 0, 'S' },
      { "AP VACC --", 0, '1' },
      { "AP VACC -1", 0, '2' },
      { "AP VACC  0", 0, '3' },
      { "AP VACC +1", 0, '4' },
      { "AP VACC ++", 0, '5' },
      { "AP AUTO", 0, '6' }
    };
    RegisterPage(mnu0, sizeof(mnu0) / sizeof(MFDBUTTONMENU));
    RegisterFunction("AP", OAPI_KEY_A, &ScramAttitude::Button_AP);
    RegisterFunction("DP-", OAPI_KEY_D, &ScramAttitude::Button_DN);
    RegisterFunction("DP+", OAPI_KEY_U, &ScramAttitude::Button_UP);
    RegisterFunction("LOG", OAPI_KEY_L, &ScramAttitude::Button_LOG);
    RegisterFunction("REA", OAPI_KEY_R, &ScramAttitude::Button_REA);
    RegisterFunction("DIA", OAPI_KEY_S, &ScramAttitude::Button_DIA);
    RegisterFunction("A--", OAPI_KEY_1, &ScramAttitude::Button_A2D);
    RegisterFunction("A1D", OAPI_KEY_2, &ScramAttitude::Button_A1D);
    RegisterFunction("A0", OAPI_KEY_3, &ScramAttitude::Button_A0);
    RegisterFunction("A1U", OAPI_KEY_4, &ScramAttitude::Button_A1U);
    RegisterFunction("A++", OAPI_KEY_5, &ScramAttitude::Button_A2U);
    RegisterFunction("APA", OAPI_KEY_6, &ScramAttitude::Button_APA);


    // Page 2, etc...
    //static const MFDBUTTONMENU mnu1[] =
    //{
    //  { "Mode Select", 0, 'M' }
    //};
    //RegisterPage(mnu1, sizeof(mnu1) / sizeof(MFDBUTTONMENU));
    //RegisterFunction("MOD", OAPI_KEY_M, &ScramAttitude::Button_MOD);

    return;
}

bool ScramAttitude_Buttons::SearchForKeysInOtherPages() const
{
    return false;
}