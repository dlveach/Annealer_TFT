// This file was generated by SquareLine Studio
// SquareLine Studio version: SquareLine Studio 1.3.2
// LVGL version: 8.3.6
// Project name: Annealer

#include "ui.h"

void ui_mainscreen_screen_init(void) 
{
  ui_mainscreen = lv_obj_create(NULL);
  lv_obj_clear_flag(ui_mainscreen, LV_OBJ_FLAG_SCROLLABLE);  /// Flags
  lv_obj_set_style_bg_color(ui_mainscreen, lv_color_hex(0x030303), LV_PART_MAIN | LV_STATE_DEFAULT);
  lv_obj_set_style_bg_opa(ui_mainscreen, 255, LV_PART_MAIN | LV_STATE_DEFAULT);

  ui_BtnAuto = lv_btn_create(ui_mainscreen);
  lv_obj_set_width(ui_BtnAuto, 80);
  lv_obj_set_height(ui_BtnAuto, 30);
  lv_obj_set_x(ui_BtnAuto, 20);
  lv_obj_set_y(ui_BtnAuto, 60);
  lv_obj_add_flag(ui_BtnAuto, LV_OBJ_FLAG_CHECKABLE | LV_OBJ_FLAG_SCROLL_ON_FOCUS);  /// Flags
  lv_obj_clear_flag(ui_BtnAuto, LV_OBJ_FLAG_SCROLLABLE);                             /// Flags
  lv_obj_set_style_bg_color(ui_BtnAuto, lv_color_hex(0x0042C3), LV_PART_MAIN | LV_STATE_DEFAULT);
  lv_obj_set_style_bg_opa(ui_BtnAuto, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
  lv_obj_set_style_shadow_color(ui_BtnAuto, lv_color_hex(0x3C3C3C), LV_PART_MAIN | LV_STATE_DEFAULT);
  lv_obj_set_style_shadow_opa(ui_BtnAuto, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
  lv_obj_set_style_shadow_ofs_x(ui_BtnAuto, 1, LV_PART_MAIN | LV_STATE_DEFAULT);
  lv_obj_set_style_shadow_ofs_y(ui_BtnAuto, 1, LV_PART_MAIN | LV_STATE_DEFAULT);
  lv_obj_set_style_bg_color(ui_BtnAuto, lv_color_hex(0xCC0000), LV_PART_MAIN | LV_STATE_CHECKED);
  lv_obj_set_style_bg_opa(ui_BtnAuto, 255, LV_PART_MAIN | LV_STATE_CHECKED);

  ui_BtnAutoLabel = lv_label_create(ui_BtnAuto);
  lv_obj_set_width(ui_BtnAutoLabel, LV_SIZE_CONTENT);   /// 1
  lv_obj_set_height(ui_BtnAutoLabel, LV_SIZE_CONTENT);  /// 1
  lv_obj_set_align(ui_BtnAutoLabel, LV_ALIGN_CENTER);
  lv_label_set_text(ui_BtnAutoLabel, "Auto");
  lv_obj_set_style_text_color(ui_BtnAutoLabel, lv_color_hex(0xFCE200), LV_PART_MAIN | LV_STATE_DEFAULT);
  lv_obj_set_style_text_opa(ui_BtnAutoLabel, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
  lv_obj_set_style_text_font(ui_BtnAutoLabel, &lv_font_montserrat_18, LV_PART_MAIN | LV_STATE_DEFAULT);

  ui_Temp = lv_label_create(ui_mainscreen);
  lv_obj_set_width(ui_Temp, 100);
  lv_obj_set_height(ui_Temp, LV_SIZE_CONTENT);  /// 1
  lv_obj_set_x(ui_Temp, 30);
  lv_obj_set_y(ui_Temp, -14);
  lv_obj_set_align(ui_Temp, LV_ALIGN_CENTER);
  lv_label_set_text(ui_Temp, "000");
  lv_obj_set_style_text_color(ui_Temp, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
  lv_obj_set_style_text_opa(ui_Temp, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
  lv_obj_set_style_text_align(ui_Temp, LV_TEXT_ALIGN_RIGHT, LV_PART_MAIN | LV_STATE_DEFAULT);
  lv_obj_set_style_text_font(ui_Temp, &lv_font_montserrat_20, LV_PART_MAIN | LV_STATE_DEFAULT);

  ui_TempLabel = lv_label_create(ui_Temp);
  lv_obj_set_width(ui_TempLabel, LV_SIZE_CONTENT);   /// 1
  lv_obj_set_height(ui_TempLabel, LV_SIZE_CONTENT);  /// 1
  lv_obj_set_x(ui_TempLabel, -25);
  lv_obj_set_y(ui_TempLabel, 0);
  lv_obj_set_align(ui_TempLabel, LV_ALIGN_CENTER);
  lv_label_set_text(ui_TempLabel, "Temp:");
  lv_obj_set_style_text_color(ui_TempLabel, lv_color_hex(0x00DCFF), LV_PART_MAIN | LV_STATE_DEFAULT);
  lv_obj_set_style_text_opa(ui_TempLabel, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
  lv_obj_set_style_text_font(ui_TempLabel, &lv_font_montserrat_14, LV_PART_MAIN | LV_STATE_DEFAULT);

  ui_Amps = lv_label_create(ui_mainscreen);
  lv_obj_set_width(ui_Amps, 100);
  lv_obj_set_height(ui_Amps, LV_SIZE_CONTENT);  /// 1
  lv_obj_set_x(ui_Amps, 30);
  lv_obj_set_y(ui_Amps, -45);
  lv_obj_set_align(ui_Amps, LV_ALIGN_CENTER);
  lv_label_set_text(ui_Amps, "000");
  lv_obj_set_style_text_color(ui_Amps, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
  lv_obj_set_style_text_opa(ui_Amps, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
  lv_obj_set_style_text_align(ui_Amps, LV_TEXT_ALIGN_RIGHT, LV_PART_MAIN | LV_STATE_DEFAULT);
  lv_obj_set_style_text_font(ui_Amps, &lv_font_montserrat_20, LV_PART_MAIN | LV_STATE_DEFAULT);

  ui_AmpsLabel = lv_label_create(ui_Amps);
  lv_obj_set_width(ui_AmpsLabel, LV_SIZE_CONTENT);   /// 1
  lv_obj_set_height(ui_AmpsLabel, LV_SIZE_CONTENT);  /// 1
  lv_obj_set_x(ui_AmpsLabel, -25);
  lv_obj_set_y(ui_AmpsLabel, 0);
  lv_obj_set_align(ui_AmpsLabel, LV_ALIGN_CENTER);
  lv_label_set_text(ui_AmpsLabel, "Amps:");
  lv_obj_clear_flag(ui_AmpsLabel, LV_OBJ_FLAG_CLICK_FOCUSABLE);  /// Flags
  lv_obj_set_style_text_color(ui_AmpsLabel, lv_color_hex(0x00DCFF), LV_PART_MAIN | LV_STATE_DEFAULT);
  lv_obj_set_style_text_opa(ui_AmpsLabel, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
  lv_obj_set_style_text_font(ui_AmpsLabel, &lv_font_montserrat_14, LV_PART_MAIN | LV_STATE_DEFAULT);

  ui_BtnManual = lv_btn_create(ui_mainscreen);
  lv_obj_set_width(ui_BtnManual, 80);
  lv_obj_set_height(ui_BtnManual, 30);
  lv_obj_set_x(ui_BtnManual, 20);
  lv_obj_set_y(ui_BtnManual, 114);
  lv_obj_add_flag(ui_BtnManual, LV_OBJ_FLAG_SCROLL_ON_FOCUS);  /// Flags
  lv_obj_clear_flag(ui_BtnManual, LV_OBJ_FLAG_SCROLLABLE);     /// Flags
  lv_obj_set_style_bg_color(ui_BtnManual, lv_color_hex(0x0042C3), LV_PART_MAIN | LV_STATE_DEFAULT);
  lv_obj_set_style_bg_opa(ui_BtnManual, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
  lv_obj_set_style_shadow_color(ui_BtnManual, lv_color_hex(0x3C3C3C), LV_PART_MAIN | LV_STATE_DEFAULT);
  lv_obj_set_style_shadow_opa(ui_BtnManual, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
  lv_obj_set_style_shadow_ofs_x(ui_BtnManual, 1, LV_PART_MAIN | LV_STATE_DEFAULT);
  lv_obj_set_style_shadow_ofs_y(ui_BtnManual, 1, LV_PART_MAIN | LV_STATE_DEFAULT);

  ui_BtnManualLabel = lv_label_create(ui_BtnManual);
  lv_obj_set_width(ui_BtnManualLabel, LV_SIZE_CONTENT);   /// 1
  lv_obj_set_height(ui_BtnManualLabel, LV_SIZE_CONTENT);  /// 1
  lv_obj_set_align(ui_BtnManualLabel, LV_ALIGN_CENTER);
  lv_label_set_text(ui_BtnManualLabel, "Manual");
  lv_obj_set_style_text_color(ui_BtnManualLabel, lv_color_hex(0xFCE200), LV_PART_MAIN | LV_STATE_DEFAULT);
  lv_obj_set_style_text_opa(ui_BtnManualLabel, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
  lv_obj_set_style_text_font(ui_BtnManualLabel, &lv_font_montserrat_18, LV_PART_MAIN | LV_STATE_DEFAULT);

  ui_State = lv_label_create(ui_mainscreen);
  lv_obj_set_width(ui_State, 250);
  lv_obj_set_height(ui_State, 30);
  lv_obj_set_x(ui_State, 0);
  lv_obj_set_y(ui_State, 10);
  lv_obj_set_align(ui_State, LV_ALIGN_TOP_MID);
  lv_label_set_text(ui_State, "Paused");
  lv_obj_set_style_text_color(ui_State, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
  lv_obj_set_style_text_opa(ui_State, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
  lv_obj_set_style_text_align(ui_State, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN | LV_STATE_DEFAULT);
  lv_obj_set_style_text_font(ui_State, &lv_font_montserrat_22, LV_PART_MAIN | LV_STATE_DEFAULT);
  lv_obj_set_style_radius(ui_State, 15, LV_PART_MAIN | LV_STATE_DEFAULT);
  lv_obj_set_style_bg_color(ui_State, lv_color_hex(0x007000), LV_PART_MAIN | LV_STATE_DEFAULT);
  lv_obj_set_style_bg_opa(ui_State, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
  lv_obj_set_style_pad_left(ui_State, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
  lv_obj_set_style_pad_right(ui_State, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
  lv_obj_set_style_pad_top(ui_State, 3, LV_PART_MAIN | LV_STATE_DEFAULT);
  lv_obj_set_style_pad_bottom(ui_State, 0, LV_PART_MAIN | LV_STATE_DEFAULT);

  ui_SliderAnnealTime = lv_slider_create(ui_mainscreen);
  lv_slider_set_range(ui_SliderAnnealTime, 10, 99);
  lv_slider_set_value(ui_SliderAnnealTime, 25, LV_ANIM_OFF);
  if (lv_slider_get_mode(ui_SliderAnnealTime) == LV_SLIDER_MODE_RANGE) lv_slider_set_left_value(ui_SliderAnnealTime, 0, LV_ANIM_OFF);
  lv_obj_set_width(ui_SliderAnnealTime, 250);
  lv_obj_set_height(ui_SliderAnnealTime, LV_SIZE_CONTENT);  /// 10
  lv_obj_set_x(ui_SliderAnnealTime, 0);
  lv_obj_set_y(ui_SliderAnnealTime, 56);
  lv_obj_set_align(ui_SliderAnnealTime, LV_ALIGN_CENTER);
  lv_obj_set_style_bg_color(ui_SliderAnnealTime, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
  lv_obj_set_style_bg_opa(ui_SliderAnnealTime, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
  lv_obj_set_style_pad_left(ui_SliderAnnealTime, 65, LV_PART_MAIN | LV_STATE_DEFAULT);
  lv_obj_set_style_pad_right(ui_SliderAnnealTime, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
  lv_obj_set_style_pad_top(ui_SliderAnnealTime, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
  lv_obj_set_style_pad_bottom(ui_SliderAnnealTime, 0, LV_PART_MAIN | LV_STATE_DEFAULT);

  lv_obj_set_style_border_color(ui_SliderAnnealTime, lv_color_hex(0x000000), LV_PART_INDICATOR | LV_STATE_DEFAULT);
  lv_obj_set_style_border_opa(ui_SliderAnnealTime, 255, LV_PART_INDICATOR | LV_STATE_DEFAULT);
  lv_obj_set_style_border_width(ui_SliderAnnealTime, 5, LV_PART_INDICATOR | LV_STATE_DEFAULT);
  lv_obj_set_style_border_side(ui_SliderAnnealTime, LV_BORDER_SIDE_FULL, LV_PART_INDICATOR | LV_STATE_DEFAULT);

  lv_obj_set_style_pad_left(ui_SliderAnnealTime, 0, LV_PART_KNOB | LV_STATE_DEFAULT);
  lv_obj_set_style_pad_right(ui_SliderAnnealTime, 0, LV_PART_KNOB | LV_STATE_DEFAULT);
  lv_obj_set_style_pad_top(ui_SliderAnnealTime, 0, LV_PART_KNOB | LV_STATE_DEFAULT);
  lv_obj_set_style_pad_bottom(ui_SliderAnnealTime, 0, LV_PART_KNOB | LV_STATE_DEFAULT);

  ui_AnnealTime = lv_label_create(ui_SliderAnnealTime);
  lv_obj_set_width(ui_AnnealTime, LV_SIZE_CONTENT);   /// 1
  lv_obj_set_height(ui_AnnealTime, LV_SIZE_CONTENT);  /// 1
  lv_obj_set_x(ui_AnnealTime, -125);
  lv_obj_set_y(ui_AnnealTime, 0);
  lv_obj_set_align(ui_AnnealTime, LV_ALIGN_CENTER);
  lv_label_set_text(ui_AnnealTime, "2.5 sec");
  lv_obj_set_style_text_color(ui_AnnealTime, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
  lv_obj_set_style_text_opa(ui_AnnealTime, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
  lv_obj_set_style_text_font(ui_AnnealTime, &lv_font_montserrat_18, LV_PART_MAIN | LV_STATE_DEFAULT);

  ui_SliderPauseTime = lv_slider_create(ui_mainscreen);
  lv_slider_set_range(ui_SliderPauseTime, 1, 10);
  lv_slider_set_value(ui_SliderPauseTime, 5, LV_ANIM_OFF);
  if (lv_slider_get_mode(ui_SliderPauseTime) == LV_SLIDER_MODE_RANGE) lv_slider_set_left_value(ui_SliderPauseTime, 0, LV_ANIM_OFF);
  lv_obj_set_width(ui_SliderPauseTime, 250);
  lv_obj_set_height(ui_SliderPauseTime, LV_SIZE_CONTENT);  /// 10
  lv_obj_set_x(ui_SliderPauseTime, 0);
  lv_obj_set_y(ui_SliderPauseTime, 98);
  lv_obj_set_align(ui_SliderPauseTime, LV_ALIGN_CENTER);
  lv_obj_set_style_bg_color(ui_SliderPauseTime, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
  lv_obj_set_style_bg_opa(ui_SliderPauseTime, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
  lv_obj_set_style_pad_left(ui_SliderPauseTime, 65, LV_PART_MAIN | LV_STATE_DEFAULT);
  lv_obj_set_style_pad_right(ui_SliderPauseTime, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
  lv_obj_set_style_pad_top(ui_SliderPauseTime, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
  lv_obj_set_style_pad_bottom(ui_SliderPauseTime, 0, LV_PART_MAIN | LV_STATE_DEFAULT);

  lv_obj_set_style_border_color(ui_SliderPauseTime, lv_color_hex(0x000000), LV_PART_INDICATOR | LV_STATE_DEFAULT);
  lv_obj_set_style_border_opa(ui_SliderPauseTime, 255, LV_PART_INDICATOR | LV_STATE_DEFAULT);
  lv_obj_set_style_border_width(ui_SliderPauseTime, 5, LV_PART_INDICATOR | LV_STATE_DEFAULT);
  lv_obj_set_style_border_side(ui_SliderPauseTime, LV_BORDER_SIDE_FULL, LV_PART_INDICATOR | LV_STATE_DEFAULT);

  lv_obj_set_style_pad_left(ui_SliderPauseTime, 0, LV_PART_KNOB | LV_STATE_DEFAULT);
  lv_obj_set_style_pad_right(ui_SliderPauseTime, 0, LV_PART_KNOB | LV_STATE_DEFAULT);
  lv_obj_set_style_pad_top(ui_SliderPauseTime, 0, LV_PART_KNOB | LV_STATE_DEFAULT);
  lv_obj_set_style_pad_bottom(ui_SliderPauseTime, 0, LV_PART_KNOB | LV_STATE_DEFAULT);

  ui_PauseTime = lv_label_create(ui_SliderPauseTime);
  lv_obj_set_width(ui_PauseTime, LV_SIZE_CONTENT);   /// 1
  lv_obj_set_height(ui_PauseTime, LV_SIZE_CONTENT);  /// 1
  lv_obj_set_x(ui_PauseTime, -125);
  lv_obj_set_y(ui_PauseTime, 0);
  lv_obj_set_align(ui_PauseTime, LV_ALIGN_CENTER);
  lv_label_set_text(ui_PauseTime, "5 sec");
  lv_obj_set_style_text_color(ui_PauseTime, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
  lv_obj_set_style_text_opa(ui_PauseTime, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
  lv_obj_set_style_text_font(ui_PauseTime, &lv_font_montserrat_18, LV_PART_MAIN | LV_STATE_DEFAULT);

  ui_Count = lv_label_create(ui_mainscreen);
  lv_obj_set_width(ui_Count, 100);
  lv_obj_set_height(ui_Count, LV_SIZE_CONTENT);  /// 1
  lv_obj_set_x(ui_Count, 30);
  lv_obj_set_y(ui_Count, 17);
  lv_obj_set_align(ui_Count, LV_ALIGN_CENTER);
  lv_label_set_text(ui_Count, "000");
  lv_obj_set_style_text_color(ui_Count, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
  lv_obj_set_style_text_opa(ui_Count, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
  lv_obj_set_style_text_align(ui_Count, LV_TEXT_ALIGN_RIGHT, LV_PART_MAIN | LV_STATE_DEFAULT);
  lv_obj_set_style_text_font(ui_Count, &lv_font_montserrat_20, LV_PART_MAIN | LV_STATE_DEFAULT);

  ui_CountLabel = lv_label_create(ui_Count);
  lv_obj_set_width(ui_CountLabel, LV_SIZE_CONTENT);   /// 1
  lv_obj_set_height(ui_CountLabel, LV_SIZE_CONTENT);  /// 1
  lv_obj_set_x(ui_CountLabel, -25);
  lv_obj_set_y(ui_CountLabel, 0);
  lv_obj_set_align(ui_CountLabel, LV_ALIGN_CENTER);
  lv_label_set_text(ui_CountLabel, "Count:");
  lv_obj_set_style_text_color(ui_CountLabel, lv_color_hex(0x00DCFF), LV_PART_MAIN | LV_STATE_DEFAULT);
  lv_obj_set_style_text_opa(ui_CountLabel, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
  lv_obj_set_style_text_font(ui_CountLabel, &lv_font_montserrat_14, LV_PART_MAIN | LV_STATE_DEFAULT);

  ui_BtnReset = lv_btn_create(ui_mainscreen);
  lv_obj_set_width(ui_BtnReset, 40);
  lv_obj_set_height(ui_BtnReset, 20);
  lv_obj_set_x(ui_BtnReset, 257);
  lv_obj_set_y(ui_BtnReset, 128);
  lv_obj_add_flag(ui_BtnReset, LV_OBJ_FLAG_SCROLL_ON_FOCUS);  /// Flags
  lv_obj_clear_flag(ui_BtnReset, LV_OBJ_FLAG_SCROLLABLE);     /// Flags
  lv_obj_set_style_bg_color(ui_BtnReset, lv_color_hex(0x0042C3), LV_PART_MAIN | LV_STATE_DEFAULT);
  lv_obj_set_style_bg_opa(ui_BtnReset, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
  lv_obj_set_style_shadow_color(ui_BtnReset, lv_color_hex(0x3C3C3C), LV_PART_MAIN | LV_STATE_DEFAULT);
  lv_obj_set_style_shadow_opa(ui_BtnReset, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
  lv_obj_set_style_shadow_ofs_x(ui_BtnReset, 1, LV_PART_MAIN | LV_STATE_DEFAULT);
  lv_obj_set_style_shadow_ofs_y(ui_BtnReset, 1, LV_PART_MAIN | LV_STATE_DEFAULT);

  ui_BtnResetLabel = lv_label_create(ui_BtnReset);
  lv_obj_set_width(ui_BtnResetLabel, LV_SIZE_CONTENT);   /// 1
  lv_obj_set_height(ui_BtnResetLabel, LV_SIZE_CONTENT);  /// 1
  lv_obj_set_align(ui_BtnResetLabel, LV_ALIGN_CENTER);
  lv_label_set_text(ui_BtnResetLabel, "Reset");
  lv_obj_set_style_text_color(ui_BtnResetLabel, lv_color_hex(0xFCE200), LV_PART_MAIN | LV_STATE_DEFAULT);
  lv_obj_set_style_text_opa(ui_BtnResetLabel, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
  lv_obj_set_style_text_font(ui_BtnResetLabel, &lv_font_montserrat_10, LV_PART_MAIN | LV_STATE_DEFAULT);

  lv_obj_add_event_cb(ui_BtnAuto, ui_event_BtnAuto, LV_EVENT_ALL, NULL);
  lv_obj_add_event_cb(ui_BtnManual, ui_event_BtnManual, LV_EVENT_ALL, NULL);
  lv_obj_add_event_cb(ui_SliderAnnealTime, ui_event_SliderAnnealTime, LV_EVENT_ALL, NULL);
  lv_obj_add_event_cb(ui_SliderPauseTime, ui_event_SliderPauseTime, LV_EVENT_ALL, NULL);
  lv_obj_add_event_cb(ui_BtnReset, ui_event_BtnReset, LV_EVENT_ALL, NULL);
}