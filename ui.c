// This file was generated by SquareLine Studio
// SquareLine Studio version: SquareLine Studio 1.3.2
// LVGL version: 8.3.6
// Project name: Annealer

#include "ui.h"
#include "ui_helpers.h"

///////////////////// VARIABLES ////////////////////


// SCREEN: ui_mainscreen
void ui_mainscreen_screen_init(void);
lv_obj_t *ui_mainscreen;
void ui_event_BtnAuto(lv_event_t *e);
lv_obj_t *ui_BtnAuto;
lv_obj_t *ui_BtnAutoLabel;
lv_obj_t *ui_Temp;
lv_obj_t *ui_TempLabel;
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
lv_obj_t *ui____initial_actions0;

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
    SliderPauseTimeChangedFn(e);
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

///////////////////// SCREENS ////////////////////

void ui_init(void) {
  LV_EVENT_GET_COMP_CHILD = lv_event_register_id();

  lv_disp_t *dispp = lv_disp_get_default();
  lv_theme_t *theme = lv_theme_default_init(dispp, lv_palette_main(LV_PALETTE_BLUE), lv_palette_main(LV_PALETTE_RED), false, LV_FONT_DEFAULT);
  lv_disp_set_theme(dispp, theme);
  ui_mainscreen_screen_init();
  ui____initial_actions0 = lv_obj_create(NULL);
  lv_disp_load_scr(ui_mainscreen);
}