/******************************************************************************
File: ui_events.ino
Supliments: Annealer_TFT2.8.ino

Organize UI event handlers here.

******************************************************************************/

/* TODO: document */
void handlePauseSliderEvent(lv_event_t * e)
{
  lv_obj_t * target = lv_event_get_target(e);
  lv_event_code_t code = lv_event_get_code(e);
  if (code == LV_EVENT_RELEASED)
  {
    int val = (int)lv_slider_get_value(target) * 1000;
    if (val != pause_time)
    {
      pause_time = val;
      writeStorage();
    }
  }
}

/* TODO: document */
void handleAnnealSliderEvent(lv_event_t * e)
{
  lv_obj_t * target = lv_event_get_target(e);
  lv_event_code_t code = lv_event_get_code(e);
  char buf[_UI_TEMPORARY_STRING_BUFFER_SIZE];
  int val = (int)lv_slider_get_value(target) * 100;
  float tmp = (float)val / 1000;
  snprintf(buf, sizeof(buf), "%2.1f sec", tmp);
  lv_label_set_text(ui_AnnealTime, buf);
  if (code == LV_EVENT_RELEASED)
  {
    anneal_time = val;
    writeStorage();
  }
}

/* TODO: document */
void handleAutoBtnEvent(lv_event_t * e)
{
  // Serial.println("handleAutoBtnEvent()");
  lv_obj_t * target = lv_event_get_target(e);
  lv_event_code_t code = lv_event_get_code(e);  
  if(code == LV_EVENT_RELEASED) {
    // Serial.println("Button Released");
    lv_state_t obj_state = lv_obj_get_state(target);
    if (obj_state & LV_STATE_CHECKED)
    {
      // Serial.println("CHECKED");
      auto_cycle_enabled = true;
      lv_label_set_text(ui_BtnAutoLabel, "Stop");
      lv_obj_add_flag(ui_BtnManual, LV_OBJ_FLAG_HIDDEN);
    }
    else
    {
      // Serial.println("NOT CHECKED");
      auto_cycle_enabled = false;
      lv_label_set_text(ui_BtnAutoLabel, "Auto");
      lv_obj_clear_flag(ui_BtnManual, LV_OBJ_FLAG_HIDDEN);
    }
  }
}

/* TODO: document */
void handleManualBtnEvent(lv_event_t * e)
{

  if (state == SYS_NOT_HOMED)  //special case for system not homed
  {
    homeFeeder();
  }
  else if (!(sys_running))
  {
    lv_event_code_t code = lv_event_get_code(e);  
    if(code == LV_EVENT_RELEASED) {
      run_once = true;
      pause_start_time = 0; //trigger immediately
    }
  }
}

/* TODO: document */
void handleResetBtnEvent(lv_event_t * e)
{
  lv_event_code_t code = lv_event_get_code(e);  
  if(code == LV_EVENT_RELEASED) {
    cycle_count = 0;
    //TODO: consider moving this to it's own button
    if (!(sys_running) && state != SYS_COOLING)
    {
      state = SYS_NOT_HOMED;
      stepper.haltAndHold();
      stepper.deenergize();
    }
  }
}

/* TODO: document */
// TODO: set slider values too!
void handleSettingsBtnEvent(lv_event_t * e)
{
  lv_event_code_t code = lv_event_get_code(e);  
  if(code == LV_EVENT_RELEASED) 
  {
    if (Serial && DEBUG) Serial.println("Handle Settings Button Release");
    // Initialize the settings display
    char buf[_UI_TEMPORARY_STRING_BUFFER_SIZE];
    lv_obj_add_state(ui_BtnSettingsSave, LV_STATE_DISABLED);
    // Feeder dwell
    snprintf(buf, sizeof(buf), "%03.1f", (float)feeder_dwell_time/1000.0);
    lv_label_set_text(ui_FeederDwellTime, buf);
    lv_slider_set_value(ui_SliderFeederDwellTime, (int)(feeder_dwell_time/100), LV_ANIM_OFF);
    // Drop offset
    snprintf(buf, sizeof(buf), "%03d", drop_pos_offset);
    lv_label_set_text(ui_DropDwellTime, buf);
    lv_slider_set_value(ui_SliderDropDwellTime, drop_pos_offset, LV_ANIM_OFF);
    // Motor speed
    snprintf(buf, sizeof(buf), "%03d", motor_speed);
    lv_label_set_text(ui_MotorSpeed, buf);
    lv_slider_set_value(ui_SliderMotorSpeed, (int)(motor_speed/10), LV_ANIM_OFF);
    // Motor current
    snprintf(buf, sizeof(buf), "%03.1f", (float)motor_current/1000.0);
    lv_label_set_text(ui_MotorCurrent, buf);
    lv_slider_set_value(ui_SliderMotorCurrent, (int)(motor_current/100), LV_ANIM_OFF);
    // High Temp Threshold
    snprintf(buf, sizeof(buf), "%03d", (int)temp_high_threshold);
    lv_label_set_text(ui_MaxTemp, buf);    
    lv_slider_set_value(ui_SliderMaxTemp, (int)temp_high_threshold, LV_ANIM_OFF);
    // Low Temp Threshold
    snprintf(buf, sizeof(buf), "%03d", (int)temp_low_threshold);
    lv_label_set_text(ui_MinTemp, buf);
    lv_slider_set_value(ui_SliderMinTemp, (int)temp_low_threshold, LV_ANIM_OFF);
  }
}

/* TODO: document */
void handleBtnSettingsCancelEvent(lv_event_t * e)
{
  if (Serial && DEBUG) Serial.println("Handle Settings Cancel");
  // Reload settings from EEPROM before returning to main screen
  readStorage();  
}

/* TODO: document */
void handleBtnSettingsSaveEvent(lv_event_t * e)
{
  if (Serial && DEBUG) Serial.println("TODO: Handle Settings Save");
  // Save settings before returning to main screen
  feeder_dwell_time = (int)lv_slider_get_value(ui_SliderFeederDwellTime) * 100;
  drop_pos_offset = (int)lv_slider_get_value(ui_SliderDropDwellTime);
  motor_speed = (int)lv_slider_get_value(ui_SliderMotorSpeed) * 10;
  stepper.setMaxSpeed(motor_speed * TIC_PULSE_MULTIPLIER);
  motor_current = (int)lv_slider_get_value(ui_SliderMotorCurrent) * 100;
  stepper.setCurrentLimit(motor_current);  
  temp_high_threshold = (int)lv_slider_get_value(ui_SliderMaxTemp);
  temp_low_threshold = (int)lv_slider_get_value(ui_SliderMinTemp);
  writeStorage();
}

/* TODO: document */
void handleSliderFeederDwellTimeEvent(lv_event_t * e)
{
  lv_obj_t * target = lv_event_get_target(e);
  lv_event_code_t code = lv_event_get_code(e);
  char buf[_UI_TEMPORARY_STRING_BUFFER_SIZE];
  int val = (int)lv_slider_get_value(target) * 100;
  float tmp = (float)val / 1000;
  snprintf(buf, sizeof(buf), "%3.1f", tmp);
  lv_label_set_text(ui_FeederDwellTime, buf);
  if (code == LV_EVENT_RELEASED) lv_obj_clear_state(ui_BtnSettingsSave, LV_STATE_DISABLED);
}

/* TODO: document */
void handleSliderDropOffsetEvent(lv_event_t * e)
{
  lv_obj_t * target = lv_event_get_target(e);
  lv_event_code_t code = lv_event_get_code(e);
  char buf[_UI_TEMPORARY_STRING_BUFFER_SIZE];
  int val = (int)lv_slider_get_value(target);
  snprintf(buf, sizeof(buf), "%03d", val);
  lv_label_set_text(ui_DropDwellTime, buf);
  if (code == LV_EVENT_RELEASED) lv_obj_clear_state(ui_BtnSettingsSave, LV_STATE_DISABLED);
}

/* TODO: document */
void handleSliderMotorSpeedEvent(lv_event_t * e)
{
  lv_obj_t * target = lv_event_get_target(e);
  lv_event_code_t code = lv_event_get_code(e);
  char buf[_UI_TEMPORARY_STRING_BUFFER_SIZE];
  int val = (int)lv_slider_get_value(target) * 10;
  snprintf(buf, sizeof(buf), "%03d", val);
  lv_label_set_text(ui_MotorSpeed, buf);
  if (code == LV_EVENT_RELEASED) lv_obj_clear_state(ui_BtnSettingsSave, LV_STATE_DISABLED);
}

/* TODO: document */
void handleSliderMotorCurrentEvent(lv_event_t * e)
{
  lv_obj_t * target = lv_event_get_target(e);
  lv_event_code_t code = lv_event_get_code(e);
  char buf[_UI_TEMPORARY_STRING_BUFFER_SIZE];
  int val = (int)lv_slider_get_value(target);
  if (val * 100 > TIC_CURRENT_LIMIT) 
  {
    val = TIC_CURRENT_LIMIT/100;
    lv_slider_set_value(ui_SliderMotorCurrent, val, LV_ANIM_OFF);
  }
  float tmp = (float)val / 10.0;
  snprintf(buf, sizeof(buf), "%3.1f", tmp);
  lv_label_set_text(ui_MotorCurrent, buf);
  if (code == LV_EVENT_RELEASED) lv_obj_clear_state(ui_BtnSettingsSave, LV_STATE_DISABLED);
}

/* TODO: document */
void handleSliderMaxTempEvent(lv_event_t * e)
{
  lv_obj_t * target = lv_event_get_target(e);
  lv_event_code_t code = lv_event_get_code(e);
  char buf[_UI_TEMPORARY_STRING_BUFFER_SIZE];
  int val = (int)lv_slider_get_value(target);
  snprintf(buf, sizeof(buf), "%03d", val);
  lv_label_set_text(ui_MaxTemp, buf);
  if (code == LV_EVENT_RELEASED) lv_obj_clear_state(ui_BtnSettingsSave, LV_STATE_DISABLED);
}

/* TODO: document */
void handleSliderMinTempEvent(lv_event_t * e)
{
  lv_obj_t * target = lv_event_get_target(e);
  lv_event_code_t code = lv_event_get_code(e);
  char buf[_UI_TEMPORARY_STRING_BUFFER_SIZE];
  int val = (int)lv_slider_get_value(target);
  snprintf(buf, sizeof(buf), "%03d", val);
  lv_label_set_text(ui_MinTemp, buf);
  if (code == LV_EVENT_RELEASED) 
  {
    //TODO: limit min temp to less than max temp
    lv_obj_clear_state(ui_BtnSettingsSave, LV_STATE_DISABLED);
  }
}


