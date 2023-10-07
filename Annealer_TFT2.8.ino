/******************************************************************************
Annealing machine usint 2.8" TFT Touch Screen Display.  (Elecrow 2.8)

References and credits:
  LVGL
    https://docs.lvgl.io/master/index.html
  TIC 500
    https://www.pololu.com/docs/0J71
  MGNZ Makes annealer project
    https://www.mgnz-makes.com

TODO:
  - save/restore last configuration state
  - TIC motor control
  - case feed cycle
  - relay control
  - temperature sensor

******************************************************************************/

#include <Arduino.h>
#include <SPI.h>
#include <EEPROM.h>

#define DEBUG true      // true: enable debug Serial output

/*_______________________ TFT TOUCH DISPLAY STUFF _______________________*/

#define Display_28        //For ESP32 3.5inch board define 'Display_35', for 2.4inch board define 'Display_24'.

#include <TFT_eSPI.h>
#include <lvgl.h>
#include "ui.h"

#if defined Display_35      // ESP32 Display 3.5inch Board
/* screen resolution */
static const uint16_t screenWidth  = 480;
static const uint16_t screenHeight = 320;
uint16_t calData[5] = { 353, 3568, 269, 3491, 7  };     //touch caldata

#elif defined Display_24    // ESP32 Display 2.4inch Board
static const uint16_t screenWidth  = 320;
static const uint16_t screenHeight = 240;
uint16_t calData[5] = { 557, 3263, 369, 3493, 3  };

#elif defined Display_28    // ESP32 Display 2.8inch Board
static const uint16_t screenWidth  = 320;
static const uint16_t screenHeight = 240;
uint16_t calData[5] = { 189, 3416, 359, 3439, 1 };
#endif

/* TFT entity */
TFT_eSPI lcd = TFT_eSPI(); 
static lv_disp_draw_buf_t draw_buf;
static lv_color_t buf1[ screenWidth * screenHeight / 13 ];

/* lvgl handler loop control */
unsigned long last_lv_loop = 0;
int lv_loop_interval = 5;

/* display flush */
void my_disp_flush( lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p )
{
  uint32_t w = ( area->x2 - area->x1 + 1 );
  uint32_t h = ( area->y2 - area->y1 + 1 );
  lcd.startWrite();
  lcd.setAddrWindow( area->x1, area->y1, w, h );
  lcd.pushColors( ( uint16_t * )&color_p->full, w * h, true );
  lcd.endWrite();
  lv_disp_flush_ready( disp );
}

/* touch read */
uint16_t touchX, touchY;
void my_touchpad_read( lv_indev_drv_t * indev_driver, lv_indev_data_t * data )
{
  bool touched = lcd.getTouch( &touchX, &touchY, 600);
  if ( !touched )
  {
    data->state = LV_INDEV_STATE_REL;
  }
  else
  {
    data->state = LV_INDEV_STATE_PR;
    /*set location*/
    data->point.x = touchX;
    data->point.y = touchY;
  }
}

/*______________________ END TFT TOUCH DISPLAY _____________________________*/

/* IC2 stuff */
#include <Wire.h>
#define I2C_SDA 22
#define I2C_SCL 21

/* State machine */
#define MANUAL 0
#define PAUSED 1
#define ANNEALING 2
#define DROPPING 3
#define COOLING 4
#define LOADING 5
#define HOMING 99

/* Defaults */
#define EEPROM_VERSION 100005           // Update when EEPROM data structure changes
#define DEFAULT_ANNEAL_TIME 2500        // Default Annealing time
#define DEFAULT_PAUSE_TIME 5000         // Default Time to pause between auto anneal cycles for cooling
#define MAX_TEMP 150                    // Default Threshold for cool down

/* TIC stepper controller */
#include <Tic.h>  
#define TIC_STEP_MODE TicStepMode::Microstep8  //TIC microstep mode.  WARNING: Many things need adjusting if changed
#define TIC_CURRENT_LIMIT 1100          //TIC current limit in milliamps (see TIC docs)
#define TIC_MAX_ACCEL 200000            //steps per second per second
#define TIC_MAX_DECEL 200000            //steps per second per second
#define TIC_DIRECTION 1                 //positive (1) for forward, negative (-1) for reverse
#define TIC_STARTING_SPEED 100          //pulses per sec (1/8 micro step, use multiplier0)
#define TIC_MAX_SPEED 300               //pulses per sec (1/8 micro step, use multiplier)
#define TIC_PULSE_MULTIPLIER 10000      // TIC uses pulses/10,000 Seconds for speed. See TIC documentation.
#define TIC_PING_INTERVAL 100           // TIC command timeout ping interval
#define TIC_PULSES_PER_REV  200 * 8     // 200 step/rev * 8 (microstepping mode)
#define PAUSE_POSITION 1200
#define DROP_POSITION 1600              
TicI2C stepper(0x0E);
unsigned long tic_ping = 0;
int motor_speed = TIC_MAX_SPEED;
int motor_current = TIC_CURRENT_LIMIT;

/* Program Globals */
int anneal_time = DEFAULT_ANNEAL_TIME;  
unsigned long previous_anneal = 0;

int pause_time = DEFAULT_PAUSE_TIME;                  
unsigned long previous_pause = 0;

int data_refresh_interval = 500;
unsigned long last_data_refresh = 0;

int state = MANUAL;
int last_state = -1;
bool run_once = false;

bool auto_cycle_enabled = false;
int cycle_count = 0;  
int last_cycle_count = cycle_count;

int sys_amps = 0;  //--------------------------> TODO: make float!!!
int sys_temp = 0;
int max_temp = MAX_TEMP;

/* Setup */
void setup()
{
  Serial.begin( 115200 );
  if (Serial && DEBUG) Serial.println( "Begin Setup ..." );

  //Wire init
  Wire.setPins(I2C_SDA, I2C_SCL);
  Wire.begin();

  //Port_D
  pinMode(25, OUTPUT);
  digitalWrite(25, LOW);

  if (Serial && DEBUG) Serial.println( "LCD Init ..." );
  //LCD init
  lcd.begin();          
  lcd.setRotation(1); 
  lcd.fillScreen(TFT_BLACK);
  lcd.setTouch(calData);
  delay(100);
  //background light pin
  pinMode(27, OUTPUT);
  digitalWrite(27, HIGH);

  //lvgl init
  lv_init();
  
  lv_disp_draw_buf_init( &draw_buf, buf1, NULL, screenWidth * screenHeight / 13 );

  //Display init
  static lv_disp_drv_t disp_drv;
  lv_disp_drv_init( &disp_drv );
  /*Display driver port of LVGL*/
  disp_drv.hor_res = screenWidth;
  disp_drv.ver_res = screenHeight;
  disp_drv.flush_cb = my_disp_flush;
  disp_drv.draw_buf = &draw_buf;
  lv_disp_drv_register( &disp_drv );

  /* touch driver port of LVGL */
  static lv_indev_drv_t indev_drv;
  lv_indev_drv_init( &indev_drv );
  indev_drv.type = LV_INDEV_TYPE_POINTER;
  indev_drv.read_cb = my_touchpad_read;
  lv_indev_drv_register( &indev_drv );
  ui_init();        //LVGL UI init
  
  if (Serial && DEBUG) Serial.println( "Storage Init ..." );
  readStorage();

  if (Serial && DEBUG) Serial.println( "TIC Stepper Controller Init ..." );
  stepper.setProduct(TicProduct::T500);
  stepper.setStepMode(TIC_STEP_MODE);  
  stepper.setCurrentLimit(motor_current);  
  stepper.setMaxAccel(TIC_MAX_ACCEL);
  stepper.setMaxDecel(TIC_MAX_DECEL);
//  stepper.setMaxSpeed(motor_speed * TIC_PULSE_MULTIPLIER);
  stepper.setMaxSpeed(TIC_MAX_SPEED * TIC_PULSE_MULTIPLIER);
  stepper.setStartingSpeed(TIC_STARTING_SPEED * TIC_PULSE_MULTIPLIER);
  stepper.setTargetVelocity(0);
  stepper.exitSafeStart();
  stepper.resetCommandTimeout();
  if (stepper.getEnergized()) stepper.deenergize();  // disable stepper until run
  if (Serial && DEBUG)
  {
    Serial.print("Stepper Current Limit: ");
    Serial.println(stepper.getCurrentLimit());
    Serial.print("Stepper Max Speed: ");
    Serial.println(stepper.getMaxSpeed());
    Serial.print("Stepper Max Accel: ");
    Serial.println(stepper.getMaxAccel());
    Serial.print("Stepper Operational State: ");
    Serial.println((unsigned int)stepper.getOperationState());
  }

  if (Serial && DEBUG) Serial.println( "Homing ..." );
  homeDropper();

  if (Serial && DEBUG) Serial.println( "Setup done" );
}

/* Main Loop */
bool test_motor_run = false; //TESTING
unsigned long last_test_tic = 0; //TESTING

void loop()
{
  unsigned long now = millis();
  if (now - last_lv_loop > lv_loop_interval)
  {
    lv_timer_handler();   //required!!!
    last_lv_loop = now;
  }

  //TESTING //////////////////////////////
  if (auto_cycle_enabled || run_once)
  {
    if (state == ANNEALING)
    {
      sys_amps = random(5, 15);
      sys_temp = random(100, 250);
    }
    else
    {
      sys_amps = 0;
      sys_temp = random(80, 120);
    }
  }
  else
  {
    sys_amps = 0;
    sys_temp = random(70,80);
  }
  //END TESTING /////////////////////////

  readTemp();

  readAmps();

  // Annealing cycle state machine
  if (auto_cycle_enabled || run_once)
  {
    if (state == PAUSED || state == MANUAL) 
    {
      if ((unsigned long)(now - previous_pause) > pause_time) 
      {
        //TODO: start an anneal cycle
        startAnnealing();
        state = ANNEALING;
        previous_anneal = now;
      }
      else if (state == MANUAL)
      {
        state = PAUSED;
      }
    }
    else if (state == ANNEALING) 
    {
      if ((unsigned long)(now - previous_anneal) > anneal_time) 
      {
        //TODO: Finish annealing
        finishAnnealing();
        cycle_count += 1;
        if (run_once)
        {
          state = MANUAL;
          run_once = false;
        }
        else
        {
          state = PAUSED;
        }
        previous_pause = now;
      }
    }
  }
  else
  {
    if (state == ANNEALING)
    {
      if ((unsigned long)(now - previous_anneal) > anneal_time) 
      {
        //TODO: Finish annealing
        finishAnnealing();
        cycle_count += 1;
        state = MANUAL;
        previous_pause = now;
      }
    }
    else
    {
      state = MANUAL;
      previous_pause = now;
    }
  }

  // Process stepper control
  checkStepper();

  // Refresh UI data
  refreshUI(now);
}

/* TODO: document */
void startAnnealing()
{
  if (Serial && DEBUG) Serial.println("startAnnealing()");
  if (stepper.getOperationState() != TicOperationState::Normal) stepper.exitSafeStart();
  if (!(stepper.getEnergized())) stepper.energize();
  //TESTING
  if (stepper.getOperationState() == TicOperationState::Normal) 
  {
    if (Serial && DEBUG)
    {
      Serial.println("TIC operational state: NORMAL");
      Serial.println("move stepper to drop case position");
    }
    stepper.setTargetVelocity(TIC_MAX_SPEED * TIC_PULSE_MULTIPLIER);
    while (stepper.getCurrentVelocity() == 0);  //Wait for movement
    int c = 0;
    while (!(isAtDropLocation()))   //TODO: move to main loop check
    {
      delay(1);
      if (++c == 50)
      {
        stepper.resetCommandTimeout();
        c = 0;
      }
    }
    Serial.print("Stepper drop position hit: ");
    Serial.println(stepper.getCurrentPosition());
    stepper.haltAndSetPosition(0); //resets home position 
  }
  else 
  { 
    if (Serial && DEBUG)
    {
      Serial.print("ERROR: TIC Operational state: ");
      Serial.println((unsigned int)stepper.getOperationState());
    }
  }
  //END TESTING
}

/* TODO: document */
void finishAnnealing()
{
  if (Serial && DEBUG) Serial.println("finishAnnealing()");
  if (stepper.getOperationState() != TicOperationState::Normal) stepper.exitSafeStart();
  if (stepper.getOperationState() == TicOperationState::Normal) 
  {
    if (Serial && DEBUG)
    {
      Serial.println("TIC operational state: NORMAL");
      Serial.println("move stepper to load case and pause position");
    }
    stepper.setTargetVelocity(TIC_MAX_SPEED * TIC_PULSE_MULTIPLIER);
    while (stepper.getCurrentVelocity() == 0);  //Wait for movement
    int c = 0;
    while (!(isAtPauseLocation()))  //TODO: move to main loop check
    {
      delay(1);
      if (++c == 50)
      {
        stepper.resetCommandTimeout();
        c = 0;
      }
    }
    stepper.haltAndHold();
    Serial.print("Stepper at hold position : ");
    Serial.println(stepper.getCurrentPosition());
  }        
}

/* Refresh UI */
void refreshUI(unsigned long now)
{
  if ((unsigned long)(now - last_data_refresh) > data_refresh_interval)
  {
    last_data_refresh = now;
    char buf[_UI_TEMPORARY_STRING_BUFFER_SIZE];
    if (state != last_state)
    {
      lv_color_t bg_color = lv_color_hex(0x0078EC);
      switch (state)
      {
        case MANUAL:
          snprintf(buf, sizeof(buf), "%s", "Manual");
          break;
        case PAUSED:
          snprintf(buf, sizeof(buf), "%s", "Paused");
          bg_color = lv_color_hex(0x006e00);
          break;
        case ANNEALING:
          snprintf(buf, sizeof(buf), "%s", "Annealing");
          bg_color = lv_color_hex(0xff7900);
          break;
        case DROPPING:
          snprintf(buf, sizeof(buf), "%s", "Dropping");
          bg_color = lv_color_hex(0xff7900);
          break;
        case COOLING:
          snprintf(buf, sizeof(buf), "%s", "Cooling Down");
          bg_color = lv_color_hex(0xdf0000);
          break;
        case LOADING:
          snprintf(buf, sizeof(buf), "%s", "Loading Case");
          bg_color = lv_color_hex(0xb300c1);
          break;
        case HOMING:
          snprintf(buf, sizeof(buf), "%s", "Homing");
          bg_color = lv_color_hex(0xb300c1);
          break;
        default:
          snprintf(buf, sizeof(buf), "%s", "Error");
          bg_color = lv_color_hex(0xdf0000);
          break;
      }
      lv_label_set_text(ui_State, buf);
      lv_obj_set_style_bg_color(ui_State, bg_color, LV_PART_MAIN | LV_STATE_DEFAULT );
      last_state = state;
    }
    if (cycle_count != last_cycle_count)
    {
      snprintf(buf, sizeof(buf), "%03d", cycle_count);
      lv_label_set_text(ui_Count, buf);
      last_cycle_count = cycle_count;
    }
    snprintf(buf, sizeof(buf), "%03d", sys_amps);
    lv_label_set_text(ui_Amps, buf);
    snprintf(buf, sizeof(buf), "%03d", sys_temp);
    lv_label_set_text(ui_Temp, buf);
  }
}

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
  lv_event_code_t code = lv_event_get_code(e);  
  if(code == LV_EVENT_RELEASED) {
    run_once = true;
    previous_pause = 0; //trigger immediately
  }
}

/* TODO: document */
void handleResetBtnEvent(lv_event_t * e)
{
  lv_event_code_t code = lv_event_get_code(e);  
  if(code == LV_EVENT_RELEASED) {
    cycle_count = 0;
  }
}

/* EEPROM stored preferences */
typedef struct storeData_t
{
  int version;
  int anneal_time;
  int pause_time;
  int motor_speed;
  int motor_current;
  int max_temp;
};
#define STORED_DATA_SIZE sizeof(storeData_t)

typedef union eeprom_buffer_t{
 storeData_t storeData;
 byte bufferData[sizeof(storeData_t)];
};
eeprom_buffer_t prefs;
#define EEPROM_BASE 0

/* TODO: document */
void readStorage()
{
  if (Serial && DEBUG) Serial.println("readStorage()");
  EEPROM.begin(1024);
  for(int i=0; i<STORED_DATA_SIZE; i++){
    prefs.bufferData[i] = byte(EEPROM.readByte(EEPROM_BASE+i));
  }
  if (prefs.storeData.version != EEPROM_VERSION)
  {
    writeStorage();
  }
  else
  {
    anneal_time = prefs.storeData.anneal_time;
    pause_time = prefs.storeData.pause_time;
    motor_speed = prefs.storeData.motor_speed;
    motor_current = prefs.storeData.motor_current;
    max_temp = prefs.storeData.max_temp;
    if (Serial && DEBUG) Serial.println("Sys data updated from EEPROM stored preferences.");
    if (Serial && DEBUG)
    {
      Serial.print("   anneal_time: ");
      Serial.println(anneal_time);
      Serial.print("   pause_time: ");
      Serial.println(pause_time);
      Serial.print("   motor_speed: ");
      Serial.println(motor_speed);
      Serial.print("   motor_current: ");
      Serial.println(motor_current);
      Serial.print("   max_temp: ");
      Serial.println(max_temp);
    }
    char buf[_UI_TEMPORARY_STRING_BUFFER_SIZE];
    snprintf(buf, sizeof(buf), "%2.1f sec", (float)anneal_time / (float)1000);
    lv_label_set_text(ui_AnnealTime, buf);
    lv_slider_set_value(ui_SliderAnnealTime, (int)anneal_time/100, LV_ANIM_OFF);
    snprintf(buf, sizeof(buf), "%d sec", (int)pause_time/1000);
    lv_label_set_text(ui_PauseTime, buf);
    lv_slider_set_value(ui_SliderPauseTime, (int)pause_time/1000, LV_ANIM_OFF);
  }
  EEPROM.end();
}

/* TODO: document */
void writeStorage()
{
  if (Serial && DEBUG) Serial.println("writeStorage()");
  EEPROM.begin(1024);
  prefs.storeData.version = EEPROM_VERSION;
  prefs.storeData.anneal_time = anneal_time;
  prefs.storeData.pause_time = pause_time;
  prefs.storeData.motor_speed = motor_speed;
  prefs.storeData.motor_current = motor_current;
  prefs.storeData.max_temp = max_temp;
  for(int i=0; i<STORED_DATA_SIZE; i++){
    EEPROM.writeByte(EEPROM_BASE+i, prefs.bufferData[i]);
  }
  EEPROM.end();
  if (Serial && DEBUG) Serial.println("Sys data written to EEPROM stored preferences.");
}

/* Home the case droper.
    Currently a test simulation, just positions the dropper at the pause position.
    Assumes at startup has been manually located to the drop position.

    Later this will utilize magnets to locate the home position.
 */
void homeDropper()
{
  if (Serial) Serial.println("TODO: homeDropper()");
  if (stepper.getOperationState() != TicOperationState::Normal) stepper.exitSafeStart();
  if (!(stepper.getEnergized())) stepper.energize();
  if (Serial && DEBUG) Serial.println("TODO: move stepper to detect magnet in home position");
  delay(50);
  Serial.println("TEST Set current stepper position to zero");
  stepper.haltAndSetPosition(0); 
  delay(50);
  Serial.print("TEST Current stepper zero position: ");
  Serial.println(stepper.getCurrentPosition());
  delay(50);
  Serial.println("TEST Set current stepper position to pause location");
  stepper.setTargetVelocity(TIC_MAX_SPEED * TIC_PULSE_MULTIPLIER);
  while (stepper.getCurrentVelocity() == 0);  //Wait for movement
  int c = 0;
  while (!(isAtPauseLocation()))
  {
    // Ping TIC command timeout every 50ms until at pause position.
    delay(1);
    if (++c == 50)
    {
      stepper.resetCommandTimeout();
      c = 0;
    }
  }
  stepper.haltAndHold();
  Serial.println("TEST Homing stopped");
  Serial.print("TEST Current stepper position: ");
  Serial.println(stepper.getCurrentPosition());
}

/* TODO: document */
bool todo_readTemp = true;
void readTemp()
{
  if (todo_readTemp) 
  {
    Serial.println("TODO: readTemp()");
    todo_readTemp = false;
  }
}

/* TODO: document */
bool todo_readAmps = true;
void readAmps()
{
  if (todo_readAmps) 
  {
    Serial.println("TODO: readAmps()");
    todo_readAmps = false;
  }
}

/*  Later this will utilize magnets to locate the position. */
bool isAtPauseLocation()
{
  if (stepper.getCurrentPosition() >= PAUSE_POSITION) return true;
  else return false;
}

/*  Later this will utilize magnets to locate the position. */
bool isAtDropLocation()
{
  if (stepper.getCurrentPosition() >= DROP_POSITION) return true;
  else return false;
}

#define PAUSED 0
#define DROP_DWELL 1
#define DROPPING 2
#define LOADING 3
int stepper_mode = PAUSED;

/* TODO: document 
    Expected to be called from main loop on every cycle.
    Manage stepper movement.
*/
void checkStepper()
{
  unsigned long now = millis();

  // ping TIC command timeout watchdog
  if (now - tic_ping > TIC_PING_INTERVAL)
  {
    stepper.resetCommandTimeout();
    tic_ping = now;
  }

  switch (stepper_mode)
  {
    case PAUSED:
      //TODO: anything?  Just wait for next cycle?
      break;
    case DROP_DWELL:
      //TODO: implement dwell for drop
      break;
    case DROPPING:
      if (isAtDropLocation()) 
      {
        Serial.print("Stepper at drop position: ");
        Serial.println(stepper.getCurrentPosition());
        stepper.haltAndSetPosition(0); //resets home position 
      }
      break;
    case LOADING:
      if (isAtPauseLocation())
      {
        stepper.haltAndHold();
        Serial.print("Stepper at pause position: ");
        Serial.println(stepper.getCurrentPosition());
      }
      break;
    default:
      if (Serial && DEBUG) Serial.println("ERROR: Unknown steper mode")
      break;
  }



  //TESTING
  if (now - last_test_tic > 5000)
  {
    if (stepper.getCurrentVelocity() == 0)
    {
      last_test_tic = now;
      unsigned int pos = stepper.getCurrentPosition();
      Serial.print("TIC current position: ");
      Serial.println(pos);
      Serial.print("Stepper Operational State: ");
      Serial.println((unsigned int)stepper.getOperationState());
    }
  }
}