// This file was generated by SquareLine Studio
// SquareLine Studio version: SquareLine Studio 1.3.2
// LVGL version: 8.3.6
// Project name: Annealer

#include "ui.h"
#include "ui_helpers.h"

///////////////////// VARIABLES ////////////////////


// SCREEN: ui_SplashScreen
void ui_SplashScreen_screen_init(void);
lv_obj_t *ui_SplashScreen;
lv_obj_t *ui_Logo;
lv_obj_t *ui_SplashVersion;
lv_obj_t *ui_SplashTitle;
lv_obj_t *ui_Spinner;


// SCREEN: ui_MainScreen
void ui_MainScreen_screen_init(void);
lv_obj_t *ui_MainScreen;
void ui_event_BtnAuto(lv_event_t *e);
lv_obj_t *ui_BtnAuto;
lv_obj_t *ui_BtnAutoLabel;
lv_obj_t *ui_Temp;
lv_obj_t *ui_TempLabel;
lv_obj_t *ui_TempFCLabel;
lv_obj_t *ui_Amps;
lv_obj_t *ui_AmpsLabel;
void ui_event_BtnManual(lv_event_t *e);
lv_obj_t *ui_BtnManual;
lv_obj_t *ui_BtnManualLabel;
lv_obj_t *ui_State;
void ui_event_SliderAnnealTime(lv_event_t *e);
lv_obj_t *ui_SliderAnnealTime;
lv_obj_t *ui_AnnealTime;
void ui_event_SliderPauseTime(lv_event_t *e);
lv_obj_t *ui_SliderPauseTime;
lv_obj_t *ui_PauseTime;
lv_obj_t *ui_Count;
lv_obj_t *ui_CountLabel;
void ui_event_BtnReset(lv_event_t *e);
lv_obj_t *ui_BtnReset;
lv_obj_t *ui_BtnResetLabel;
void ui_event_BtnSettings(lv_event_t *e);
lv_obj_t *ui_BtnSettings;
lv_obj_t *ui_BtnSettingsLabel;
lv_obj_t *ui_AnnealTimeLabel;
lv_obj_t *ui_PauseTimeLabel;


// SCREEN: ui_SettingsScreen
void ui_SettingsScreen_screen_init(void);
lv_obj_t *ui_SettingsScreen;
void ui_event_BtnSettingsCancel(lv_event_t *e);
lv_obj_t *ui_BtnSettingsCancel;
lv_obj_t *ui_BtnSettingsCancelLabel;
void ui_event_BtnSettingsSave(lv_event_t *e);
lv_obj_t *ui_BtnSettingsSave;
lv_obj_t *ui_BtnSettingsSaveLabel;
void ui_event_SliderFeederDwellTime(lv_event_t *e);
lv_obj_t *ui_SliderFeederDwellTime;
lv_obj_t *ui_FeederDwellTime;
lv_obj_t *ui_FeederDwellTimeLabel;
void ui_event_SliderDropDwellTime(lv_event_t *e);
lv_obj_t *ui_SliderDropDwellTime;
lv_obj_t *ui_DropDwellTime;
lv_obj_t *ui_DropDwellTimeLabel;
void ui_event_SliderMotorSpeed(lv_event_t *e);
lv_obj_t *ui_SliderMotorSpeed;
lv_obj_t *ui_MotorSpeed;
lv_obj_t *ui_MotorSpeedLabel;
void ui_event_SliderMotorCurent(lv_event_t *e);
lv_obj_t *ui_SliderMotorCurrent;
lv_obj_t *ui_MotorCurrent;
lv_obj_t *ui_MotorCurrentLabel;
void ui_event_SliderMaxTemp(lv_event_t *e);
lv_obj_t *ui_SliderMaxTemp;
lv_obj_t *ui_MaxTemp;
lv_obj_t *ui_MaxTempLabel;
void ui_event_SliderMinTemp(lv_event_t *e);
lv_obj_t *ui_SliderMinTemp;
lv_obj_t *ui_MinTemp;
lv_obj_t *ui_MinTempLabel;
lv_obj_t *ui____initial_actions0;
const lv_img_dsc_t *ui_imgset_1608979085[1] = { &ui_img_1945259780 };
const lv_img_dsc_t *ui_imgset_stallionarms_240x[1] = { &ui_img_stallionarms_240x99_png };

///////////////////// TEST LVGL SETTINGS ////////////////////
#if LV_COLOR_DEPTH != 16
#error "LV_COLOR_DEPTH should be 16bit to match SquareLine Studio's settings"
#endif
#if LV_COLOR_16_SWAP != 0
#error "LV_COLOR_16_SWAP should be 0 to match SquareLine Studio's settings"
#endif

///////////////////// ANIMATIONS ////////////////////

///////////////////// FUNCTIONS ////////////////////
void ui_event_BtnAuto(lv_event_t *e) {
  lv_event_code_t event_code = lv_event_get_code(e);
  lv_obj_t *target = lv_event_get_target(e);
  if (event_code == LV_EVENT_RELEASED) {
    autoBtnFn(e);
  }
}
void ui_event_BtnManual(lv_event_t *e) {
  lv_event_code_t event_code = lv_event_get_code(e);
  lv_obj_t *target = lv_event_get_target(e);
  if (event_code == LV_EVENT_RELEASED) {
    manualBtnFn(e);
  }
}
void ui_event_SliderAnnealTime(lv_event_t *e) {
  lv_event_code_t event_code = lv_event_get_code(e);
  lv_obj_t *target = lv_event_get_target(e);
  if (event_code == LV_EVENT_VALUE_CHANGED) {
    SliderAnnealTimeChangedFn(e);
  }
  if (event_code == LV_EVENT_RELEASED) {
    SliderAnnealTimeChangedFn(e);
  }
}
void ui_event_SliderPauseTime(lv_event_t *e) {
  lv_event_code_t event_code = lv_event_get_code(e);
  lv_obj_t *target = lv_event_get_target(e);
  if (event_code == LV_EVENT_VALUE_CHANGED) {
    _ui_slider_set_text_value(ui_PauseTime, target, "", " sec");
    // SliderPauseTimeChangedFn(e);
  }
  if (event_code == LV_EVENT_RELEASED) {
    SliderPauseTimeChangedFn(e);
  }
}
void ui_event_BtnReset(lv_event_t *e) {
  lv_event_code_t event_code = lv_event_get_code(e);
  lv_obj_t *target = lv_event_get_target(e);
  if (event_code == LV_EVENT_RELEASED) {
    resetBtnFn(e);
  }
}
void ui_event_BtnSettings(lv_event_t *e) {
  lv_event_code_t event_code = lv_event_get_code(e);
  lv_obj_t *target = lv_event_get_target(e);
  if (event_code == LV_EVENT_RELEASED) {
    _ui_screen_change(&ui_SettingsScreen, LV_SCR_LOAD_ANIM_FADE_ON, 500, 0, &ui_SettingsScreen_screen_init);
    BtnSettingsFn(e);
  }
}
void ui_event_BtnSettingsCancel(lv_event_t *e) {
  lv_event_code_t event_code = lv_event_get_code(e);
  lv_obj_t *target = lv_event_get_target(e);
  if (event_code == LV_EVENT_RELEASED) {
    _ui_screen_change(&ui_MainScreen, LV_SCR_LOAD_ANIM_FADE_ON, 500, 0, &ui_MainScreen_screen_init);
    BtnSettingsCancelFn(e);
  }
}
void ui_event_BtnSettingsSave(lv_event_t *e) {
  lv_event_code_t event_code = lv_event_get_code(e);
  lv_obj_t *target = lv_event_get_target(e);
  if (event_code == LV_EVENT_RELEASED) {
    _ui_screen_change(&ui_MainScreen, LV_SCR_LOAD_ANIM_FADE_ON, 500, 0, &ui_MainScreen_screen_init);
    BtnSettingsSaveFn(e);
  }
}
void ui_event_SliderFeederDwellTime(lv_event_t *e) {
  lv_event_code_t event_code = lv_event_get_code(e);
  lv_obj_t *target = lv_event_get_target(e);
  if (event_code == LV_EVENT_VALUE_CHANGED) {
    SliderFeederDwellTimeFn(e);
  }
  if (event_code == LV_EVENT_RELEASED) {
    SliderFeederDwellTimeFn(e);
  }
}
void ui_event_SliderDropDwellTime(lv_event_t *e) {
  lv_event_code_t event_code = lv_event_get_code(e);
  lv_obj_t *target = lv_event_get_target(e);
  if (event_code == LV_EVENT_VALUE_CHANGED) {
    SliderDropTimeFn(e);
  }
  if (event_code == LV_EVENT_RELEASED) {
    SliderDropTimeFn(e);
  }
}
void ui_event_SliderMotorSpeed(lv_event_t *e) {
  lv_event_code_t event_code = lv_event_get_code(e);
  lv_obj_t *target = lv_event_get_target(e);
  if (event_code == LV_EVENT_VALUE_CHANGED) {
    SliderMotorSpeedFn(e);
  }
  if (event_code == LV_EVENT_RELEASED) {
    SliderMotorSpeedFn(e);
  }
}
void ui_event_SliderMotorCurent(lv_event_t *e) {
  lv_event_code_t event_code = lv_event_get_code(e);
  lv_obj_t *target = lv_event_get_target(e);
  if (event_code == LV_EVENT_VALUE_CHANGED) {
    SliderMotorCurrentFn(e);
  }
  if (event_code == LV_EVENT_RELEASED) {
    SliderMotorCurrentFn(e);
  }
}
void ui_event_SliderMaxTemp(lv_event_t *e) {
  lv_event_code_t event_code = lv_event_get_code(e);
  lv_obj_t *target = lv_event_get_target(e);
  if (event_code == LV_EVENT_VALUE_CHANGED) {
    SliderMaxTempFn(e);
  }
  if (event_code == LV_EVENT_RELEASED) {
    SliderMaxTempFn(e);
  }
}
void ui_event_SliderMinTemp(lv_event_t *e) {
  lv_event_code_t event_code = lv_event_get_code(e);
  lv_obj_t *target = lv_event_get_target(e);
  if (event_code == LV_EVENT_VALUE_CHANGED) {
    SliderMinTempFn(e);
  }
  if (event_code == LV_EVENT_RELEASED) {
    SliderMinTempFn(e);
  }
}

///////////////////// SCREENS ////////////////////

void ui_init(void) {
  LV_EVENT_GET_COMP_CHILD = lv_event_register_id();

  lv_disp_t *dispp = lv_disp_get_default();
  lv_theme_t *theme = lv_theme_default_init(dispp, lv_palette_main(LV_PALETTE_BLUE), lv_palette_main(LV_PALETTE_RED), false, LV_FONT_DEFAULT);
  lv_disp_set_theme(dispp, theme);
  ui_SplashScreen_screen_init();
  ui_MainScreen_screen_init();
  ui_SettingsScreen_screen_init();
  ui____initial_actions0 = lv_obj_create(NULL);
  lv_disp_load_scr(ui_SplashScreen);
}
