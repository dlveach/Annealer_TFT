/******************************************************************************
Annealing machine usint 2.8" TFT Touch Screen Display.  (Elecrow 2.8)

References and credits:
  Elecrow ESP32 Touchscreen 2.8 inch TFT LCD
    https://www.amazon.com/ELECROW-Touchscreen-Monitor-ILI9341V-ESP32-WROOM-32/dp/B0C8T9M2N8
  LVGL
    https://docs.lvgl.io/master/index.html
  TIC 500
    https://www.pololu.com/docs/0J71
  MGNZ Makes annealer project
    https://www.mgnz-makes.com
  XICOOLEE MCP23017 I/O Expansion Board Module
    https://www.amazon.com/dp/B0BTRMT7NP
    https://seengreat.com/product/1396/sg-io-e017


I2C Addresses:
  0x0E: Pololu TIC 500
  0x27: XICOOLEE MCP23017 I/O Expander
  0x48: Elecrow Crowtail Temperature sensor TMP102

NOTE:
Arduino IDE Tools settings for Elecrow 3.5" touch screen:
  Board: ESP32 Dev Module
  Upload speed: Max 460800
  Partition scheme: Huge APP (3mb No OTA/1MB SPIFFS)
  Flash mode: QIO
  Flash size: 4MB (32Mb)


TODO:
  - MVP: Temperature sensor cooling mode limit check.
  - MVP: Read amps.
  - Settings option for temp in Centigrade.

IDEAS:
  - Maybe Count up time in display during anneal
  - Maybe Count down time in display during pause
  - Maybe add a new button for clear homed (don't use reset)?
  
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
// static const uint16_t screenWidth  = 320;
// static const uint16_t screenHeight = 240;
// uint16_t calData[5] = { 189, 3416, 359, 3439, 1 };

static const uint16_t screenWidth  = 240;   //swap dimensions for portrait mode
static const uint16_t screenHeight = 320;
uint16_t calData[5] = { 321, 3507, 183, 3520, 4 };
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

/* TMP102 Temperature Sensor */
#include <SparkFunTMP102.h>
TMP102 tmp;

/* MCP23017 I/O expander  */
#include <Adafruit_MCP23X17.h>
Adafruit_MCP23X17 mcp;
#define MCP_DROP_RELAY_PIN  3           // MCP expander pin for 12v drop relay (output)
#define MCP_ANNEAL_SSR_PIN  15          // MCP expander pin for 48v Annealer SSR (output)
#define MCP_HALL_SENSOR_PIN 9           // MCP expander pin for Hall Effect sensor data line (input)

/* Defaults */
#define EEPROM_VERSION 100005           // Update when EEPROM data structure defaults changes
#define SPLASH_DELAY 5000               // Time to display splash screen
#define SW_VERSION "alpha_0.1.0"        // Version string
#define DEFAULT_ANNEAL_TIME 2500        // Default Annealing time
#define DEFAULT_PAUSE_TIME 3000         // Default time to pause between auto anneal cycles for cooling
#define DEFAULT_FEEDER_DWELL_TIME 1000  // Default time to dwell for case feed/drop.
#define TEMP_HIGH_THRESHOLD 150         // Default threshold temp for enter cool down (Deg F)
#define TEMP_LOW_THRESHOLD 140          // Default threshold temp for exit cool down (Deg F)
#define DEFAULT_DROP_POS_OFFSET 0       // Default offset from hall sensor trigger for drop position

/* EEPROM stored preferences */
typedef struct storeData_t
{
  int version;
  int anneal_time;
  int pause_time;
  int feeder_dwell_time;
  int drop_pos_offset;
  int motor_speed;
  int motor_current;
  int temp_high_threshold;
  int temp_low_threshold;
};
#define STORED_DATA_SIZE sizeof(storeData_t)
// Data buffer for EEPROM read/write
typedef union eeprom_buffer_t{
 storeData_t storeData;
 byte bufferData[sizeof(storeData_t)];
};
eeprom_buffer_t prefs;
#define EEPROM_BASE 0

/* TIC stepper controller */
#include <Tic.h>  
#define TIC_STEP_MODE TicStepMode::Microstep8  //TIC microstep mode.  WARNING: Many things need adjusting if changed
#define TIC_CURRENT_LIMIT 1100          //TIC current limit in milliamps (see TIC docs)
#define TIC_MAX_ACCEL 200000            //steps per second per second
#define TIC_MAX_DECEL 1000000           //steps per second per second, affects wheel stop position.
#define TIC_STARTING_SPEED 100          //pulses per sec (1/8 micro step, use multiplier0)
#define TIC_MAX_SPEED 300               //pulses per sec (1/8 micro step, use multiplier)
#define TIC_PULSE_MULTIPLIER 10000      // TIC uses pulses/10,000 Seconds for speed. See TIC documentation.
#define TIC_PING_INTERVAL 100           // TIC command timeout ping interval
#define TIC_PULSES_PER_REV  200 * 8     // 200 step/rev * 8 (microstepping mode)
#define PAUSE_POSITION 1200             // Distance (steps) to travel to pause position.
#define START_STOP_TIMEOUT 2000         // Timeout (milliseconds) to allow stepper to start moving or stop moving.  Note: keep in mind TIC_MAX_DECEL
TicI2C stepper(0x0E);
unsigned long tic_ping = 0;
int motor_speed = TIC_MAX_SPEED;
int motor_current = TIC_CURRENT_LIMIT;

//Feeder state machine
#define FEEDER_PAUSED 0
#define FEEDER_DWELL 1
#define FEEDER_DROPPING 2
#define FEEDER_LOADING 3
#define FEEDER_HOMING  99
int feeder_mode = FEEDER_PAUSED;

//Feeder Homing states
#define FEEDER_NOT_HOMED 0
#define FEEDER_HOMING_TO_PAUSE 1
#define FEEDER_HOMING_DROP_CASE 2
#define FEEDER_HOMED 3
int feeder_homed = FEEDER_NOT_HOMED;

/* System state machine */
#define SYS_MANUAL 0
#define SYS_PAUSED 1
#define SYS_FEED_CASE 2
#define SYS_ANNEALING 3
#define SYS_CASE_DROP 4
#define SYS_LOADING 5
#define SYS_COOLING 10
#define SYS_NOT_HOMED 20
#define SYS_HOMING 25
#define SYS_ERROR 99
int state = SYS_NOT_HOMED;
int last_state = -1;
bool run_once = false;
bool sys_running = false;
bool auto_cycle_enabled = false;

/* Program Timers and Globals */
int anneal_time = DEFAULT_ANNEAL_TIME;  
unsigned long anneal_start_time = 0;

int pause_time = DEFAULT_PAUSE_TIME;                  
unsigned long pause_start_time = 0;

int feeder_dwell_time = DEFAULT_FEEDER_DWELL_TIME;
unsigned long feeder_dwell_start_time = 0;
unsigned long drop_dwell_start_time = 0;

int data_refresh_interval = 500;
unsigned long last_data_refresh = 0;

int temp_polling_interval = 1000;
unsigned long last_temp_poll = 0;
float sys_temp_F = 0.0;
// float tempC = 0.0;   //TODO: option for deg C
bool sys_alert_temp = false;
int temp_high_threshold = TEMP_HIGH_THRESHOLD;
int temp_low_threshold = TEMP_LOW_THRESHOLD;

int cycle_count = 0;  
int last_cycle_count = cycle_count;  //TODO: is this really needed?  Used in display update.

int splash_delay = SPLASH_DELAY;
unsigned long splash_time = 0;
bool show_splash = true;

int drop_pos_offset = DEFAULT_DROP_POS_OFFSET;

int sys_amps = 0;  //--------------------------> TODO: make float!!!

bool tic_e = false;

/* Setup */
void setup()
{
  Serial.begin( 115200 );
  if (Serial && DEBUG) Serial.println( "Begin Setup ..." );

  //Wire init
  Wire.setPins(I2C_SDA, I2C_SCL);
  Wire.begin();

  //Port_D
  pinMode(25, OUTPUT);    //TODO: ADC for current detection?
  digitalWrite(25, LOW);

  if (Serial && DEBUG) Serial.println( "LCD Init ..." );
  //LCD init
  lcd.begin();          
  // lcd.setRotation(1);  
  lcd.setRotation(2);     // 0: portrait USB bottom, 1: landscape USB right, 2: portrait USB top, 3: landscape USB left

  // Calibrate the touch screen and retrieve the scaling factors
  /* USE DURING DEVELOPMENT ONLY
  Serial.println("Running touch_calibrate()");
  //background light pin
  pinMode(27, OUTPUT);
  digitalWrite(27, HIGH);
  touch_calibrate();
  Serial.println("End setup()");
  */

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
  
  //TMP102 Temp Sensor init
  if (!(tmp.begin(0x48, Wire)))
  {
    Serial.println("Cannot connect to TMP102");
    while(1);
  }
  Serial.println("Connected to TMP102");
  // 0-3: 0:1 fault, 1:2 faults, 2:4 faults, 3:6 faults.
  tmp.setFault(0);  // Trigger alarm immediately
  // set the polarity of the Alarm. (0:Active LOW, 1:Active HIGH).
  tmp.setAlertPolarity(1); // Active HIGH
  // set the sensor in Comparator Mode (0) or Interrupt Mode (1).
  tmp.setAlertMode(0); // Comparator Mode.
  // set the Conversion Rate (how quickly the sensor gets a new reading)
  //0-3: 0:0.25Hz, 1:1Hz, 2:4Hz, 3:8Hz
  tmp.setConversionRate(2);
  //set Extended Mode.
  //0:12-bit Temperature(-55C to +128C) 1:13-bit Temperature(-55C to +150C)
  tmp.setExtendedMode(1);
  //set T_HIGH, the upper limit to trigger the alert on
  // tmp.setHighTempF(TEMP_HIGH_THRESHOLD);  // set T_HIGH in F     TODO: remove, handled in readStorage();
  //set T_LOW, the lower limit to shut turn off the alert
  // tmp.setLowTempF(TEMP_LOW_THRESHOLD);  // set T_LOW in F     TODO: remove, handled in readStorage();

  //Storage init
  if (Serial && DEBUG) Serial.println( "Storage Init ..." );
  readStorage();

  // MCP23017 init
  if (!mcp.begin_I2C(0x27, &Wire)) {
    Serial.println("ERROR: cannot connect to MCP23017");
    while (1);
  }
  Serial.println("Connected to MCP23017");  
  mcp.pinMode(MCP_DROP_RELAY_PIN, OUTPUT);
  mcp.digitalWrite(MCP_DROP_RELAY_PIN, LOW);
  mcp.pinMode(MCP_ANNEAL_SSR_PIN, OUTPUT);
  mcp.digitalWrite(MCP_ANNEAL_SSR_PIN, LOW);
  mcp.pinMode(MCP_HALL_SENSOR_PIN, INPUT_PULLUP);

  //TIC stepper controller init
  byte error;
  Wire.beginTransmission(0x0E);
  error = Wire.endTransmission();
  if (error == 0) 
  {
    if (Serial && DEBUG) Serial.println("TIC controller found at addr 0x0E");
    tic_e = true;
  }
  else
  {
    if (Serial && DEBUG) Serial.println("No TIC found at addr 0x0E ... NO STEPPER!!!");
  }
  if (tic_e)
  {
    if (Serial && DEBUG) Serial.println( "TIC Stepper Controller Init ..." );
    stepper.setProduct(TicProduct::T500);
    stepper.setStepMode(TIC_STEP_MODE);  
    stepper.setCurrentLimit(motor_current);  
    stepper.setMaxAccel(TIC_MAX_ACCEL);
    stepper.setMaxDecel(TIC_MAX_DECEL);
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
  }
  else state = SYS_ERROR;

  if (Serial && DEBUG) Serial.println( "Setup done" );
  lv_label_set_text(ui_SplashVersion, SW_VERSION);  
  show_splash = true;
  splash_time = millis();
}

// Code to run a touch calibration, not needed when calibration values set in setup()
/* USE ONLY DURING DEVELOPMENT 
  void touch_calibrate()
  {
    uint16_t calData[5];
    uint8_t calDataOK = 0;

    // Calibrate
    lcd.fillScreen(TFT_GREEN);
    lcd.setCursor(20, 0);
    lcd.setTextFont(2);
    lcd.setTextSize(1);
    lcd.setTextColor(TFT_WHITE, TFT_BLACK);

    lcd.println("Touch corners as indicated");

    lcd.setTextFont(1);
    lcd.println();

    lcd.calibrateTouch(calData, TFT_MAGENTA, TFT_BLACK, 15);

    Serial.println(); Serial.println();
    Serial.println("// Use this calibration code in setup():");
    Serial.print("  uint16_t calData[5] = ");
    Serial.print("{ ");

    for (uint8_t i = 0; i < 5; i++)
    {
      Serial.print(calData[i]);
      if (i < 4) Serial.print(", ");
    }

    Serial.println(" };");
    Serial.print("  lcd.setTouch(calData);");
    Serial.println(); Serial.println();

    lcd.fillScreen(TFT_BLACK);
    
    lcd.setTextColor(TFT_GREEN, TFT_BLACK);
    lcd.println("Calibration complete!");
    lcd.println("Calibration code sent to Serial port.");

    delay(4000);
  }
  // Temporary test loop when calibrating touch
  void loop(void) {
    uint16_t x = 0, y = 0; // To store the touch coordinates

    // Pressed will be set true is there is a valid touch on the screen
    bool pressed = lcd.getTouch(&x, &y);

    // Draw a white spot at the detected coordinates
    if (pressed) {
      lcd.fillCircle(x, y, 2, TFT_WHITE);
      Serial.print("x,y = ");
      Serial.print(x);
      Serial.print(",");
      Serial.println(y);
    }
  }
*/

/* Main Loop */
void loop()
{
  unsigned long now = millis();
  if (now - last_lv_loop > lv_loop_interval)
  {
    lv_timer_handler();  //lvgl display driver handler
    last_lv_loop = now;
  }

  // Splash screen only for a time, then go on with main screen and complete loop
  if (show_splash && now - splash_time < splash_delay) return;
  if (show_splash) lv_disp_load_scr(ui_MainScreen);   //TODO use _ui_screen_change() ???
  show_splash = false;

  readTemp(now);
  readAmps(); //----------> TODO: implement this
  //TODO: implement over amps limit check

  // System state machine
  switch (state)
  {
    case SYS_NOT_HOMED:
      //noop - waiting to home system
      break;
    case SYS_HOMING:
      if (feeder_homed == FEEDER_HOMED)
      {
        if (Serial && DEBUG) Serial.println("End system homing, enter PAUSED/MANUAL state");
        if (auto_cycle_enabled) state = SYS_PAUSED;
        else state = SYS_MANUAL;
      }
      break;
    case SYS_PAUSED:
    case SYS_MANUAL:
      //check first for over temp.
      if (state != SYS_COOLING and sys_alert_temp)
      {
        if (Serial && DEBUG) Serial.println("Enter system cooling state");
        state = SYS_COOLING;
      }
      else if (auto_cycle_enabled || run_once)
      {
        if (!(sys_running) && now - pause_start_time > pause_time) 
        {
          if (feeder_mode == FEEDER_PAUSED && feeder_homed == FEEDER_HOMED)
          {
            if (Serial && DEBUG) Serial.println("Enter system running state, start a cycle");
            startStepper();
            sys_running = true;
            state = SYS_FEED_CASE;
            feeder_mode = FEEDER_DROPPING;
            if (run_once) run_once = false;
          }         
        }
      }
      break;
    case SYS_FEED_CASE:
      if (feeder_mode == FEEDER_LOADING)  //case has been fed by stepper
      {
        if (Serial && DEBUG) Serial.println("Case fed, enter annealing state");
        mcp.digitalWrite(MCP_ANNEAL_SSR_PIN, HIGH);  //activate the annealer relay
        anneal_start_time = now;
        state = SYS_ANNEALING;
        Serial.print("FEEDER: at position: ");
        Serial.println(stepper.getCurrentPosition());
      }
      break;
    case SYS_ANNEALING:
      if (now - anneal_start_time > anneal_time)
      {
        if (Serial && DEBUG) Serial.println("End annealing, enter case drop state");
        mcp.digitalWrite(MCP_ANNEAL_SSR_PIN, LOW);  //deactivate the annealer relay
        mcp.digitalWrite(MCP_DROP_RELAY_PIN, HIGH);  //activate the case drop solenoid relay
        drop_dwell_start_time = now;
        state = SYS_CASE_DROP;
      }
      break;
    case SYS_CASE_DROP:
      if (now - drop_dwell_start_time > feeder_dwell_time)
      {
        mcp.digitalWrite(MCP_DROP_RELAY_PIN, LOW);  //deactivate the case drop solenoid relay
        if (feeder_mode == FEEDER_PAUSED)  //drop dwell ended after feeder reached paused state
        {
          if (Serial && DEBUG) Serial.println("End of cycle, enter PAUSED/MANUAL state");
          if (auto_cycle_enabled) state = SYS_PAUSED;
          else state = SYS_MANUAL;
          sys_running = false;
          cycle_count++;
          pause_start_time = now;
        }
        else if (feeder_mode == FEEDER_LOADING)  //drop dwell ended while feeder still loading case
        {
          if (Serial && DEBUG) Serial.println("End case drop, enter system loading state");
          state = SYS_LOADING;
        }
        else
        {
          state = SYS_ERROR;
          if (Serial) 
          {
            Serial.print("ERROR: loop() bad feeder_mode: ");
            Serial.println(feeder_mode);
          }
        }
      }
      break;
    case SYS_LOADING:
      if (feeder_mode == FEEDER_PAUSED)
      {
        if (Serial && DEBUG) Serial.println("End of cycle, enter PAUSED/MANUAL state");
        if (auto_cycle_enabled) state = SYS_PAUSED;
        else state = SYS_MANUAL;
        sys_running = false;
        cycle_count++;
        pause_start_time = now;
      }
      break;
    case SYS_COOLING:
        if (!(sys_alert_temp))
        {
          if (Serial && DEBUG) Serial.println("End of cooling, enter PAUSED/MANUAL state");
          if (auto_cycle_enabled) 
          {
            state = SYS_PAUSED;
          }
          else 
          {
            run_once = false;
            state = SYS_MANUAL;
          }
        }
      break;
    case SYS_ERROR:
      stepper.haltAndHold();
      stepper.deenergize();
      while (true) delay(1000);
      break;
    default:
      if (Serial) 
      {
        Serial.print("ERROR: loop() unhandled state: "); 
        Serial.println(state);
      }
      state = SYS_ERROR;
      break;
  }

  checkStepper();   // Process stepper control
  refreshUI(now);   // Refresh UI data
}
/**/

/* Start homing the case feeder. */
void homeFeeder()
{
  if (Serial) Serial.println("homeFeeder()");
  feeder_mode = FEEDER_HOMING;
  feeder_homed = FEEDER_NOT_HOMED;
  startStepper();
  state = SYS_HOMING;
}

/* Read system temp sensor, set alert if over threshold. */
void readTemp(unsigned long now)
{
  if (now - last_temp_poll > temp_polling_interval)
  {
    tmp.wakeup();
    sys_temp_F = tmp.readTempF();
    // tempC = tmp.readTempC();   //TODO: option for deg C
    sys_alert_temp = tmp.alert();
    tmp.sleep();
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
  //TESTING SIMULATION //////////////////
    if (auto_cycle_enabled || run_once)
    {
      if (state == SYS_ANNEALING)
      {
        sys_amps = random(5, 15);
      }
      else
      {
        sys_amps = 0;
      }
    }
    else
    {
      sys_amps = 0;
    }
  //END TESTING /////////////////////////
}

/*  Return true if at pause location dist from home. */
bool isAtPauseLocation()
{
  if (!(tic_e)) return true;
  if (stepper.getCurrentPosition() >= PAUSE_POSITION) return true;    
  return false;
}

/*  Drop location located when hall sensor goes HIGH from magnet. 
    This is also the homing position.
    Position is offset by 'drop_pos_offset' steps after sensor goes high (tuning).
    Return true if sensor is HIGH and at position offset.  Otherwise return false.
*/
bool isAtDropLocation()
{
  if (!(tic_e)) return true;
  if (mcp.digitalRead(MCP_HALL_SENSOR_PIN)) 
  {
    int pos = stepper.getCurrentPosition();
    while (stepper.getCurrentPosition() - pos < drop_pos_offset);   // <---------- HANG: TODO: fix hang point?  Can it hang?
    return true;
  }
  else return false;
}

/* Manage stepper movement.  Expected to be called from main loop on every cycle. */
void checkStepper()
{
  if (!(tic_e)) return;

  unsigned long now = millis();
  
  if (now - tic_ping > TIC_PING_INTERVAL)  // ping TIC command timeout watchdog
  {
    stepper.resetCommandTimeout(); 
    tic_ping = now;
  }

  switch (feeder_mode)
  {
    case FEEDER_HOMING:
      switch (feeder_homed)
      {
        case FEEDER_NOT_HOMED:
          if (isAtDropLocation()) 
          {
            stopStepper(true);  //sets stepper position to 0
            if (Serial && DEBUG) 
            { 
              Serial.print("FEEDER: homing at drop position: ");
              Serial.print(stepper.getCurrentPosition());
              Serial.println(".  Open trap door and start drop timer.");
            }
            feeder_dwell_start_time = now;
            feeder_mode = FEEDER_DWELL;  //HACK: convoluted, reusing some modes during homing. :(
          }
          break;
        case FEEDER_HOMING_TO_PAUSE:
          if (isAtPauseLocation()) 
          {
            stopStepper(false);
            if (Serial && DEBUG) 
            {
              Serial.print("FEEDER: homing at loaded & pause position: "); 
              Serial.println(stepper.getCurrentPosition());
            }
            mcp.digitalWrite(MCP_DROP_RELAY_PIN, HIGH); //activate drop relay
            drop_dwell_start_time = now;
            feeder_mode = FEEDER_HOMING; //HACK: convoluted, reusing some modes during homing. :(
            feeder_homed = FEEDER_HOMING_DROP_CASE;
          }
          break;
        case FEEDER_HOMING_DROP_CASE:
          if (now - drop_dwell_start_time > feeder_dwell_time)
          {
            if (Serial && DEBUG) Serial.println("FEEDER: end of drop dwell, feeder homed."); 
            mcp.digitalWrite(MCP_DROP_RELAY_PIN, LOW);  //deactivate droper solenoid
            feeder_mode = FEEDER_PAUSED;
            feeder_homed = FEEDER_HOMED;
          }
          break;
        case FEEDER_HOMED:
          //do nothing, just an indicator for system state machine
          break;
        default:
          if (Serial) 
          {
            Serial.print("ERROR: FEEDER: checkStepper(): unhandled feeder_homed value: "); 
            Serial.println(feeder_homed);
          }
          break;
      }
      break;
    case FEEDER_PAUSED:
      //do nothing.  Just waiting for next cycle start.
      break;
    case FEEDER_DWELL:
      if (now - feeder_dwell_start_time > feeder_dwell_time)
      {
        startStepper();
        if (feeder_homed == FEEDER_HOMED) //little convoluted, reusing some modes during homing
        {
          Serial.println("FEEDER: Drop Dwell ended.  Start Loading.");
          Serial.print("FEEDER: at position: ");
          Serial.println(stepper.getCurrentPosition());
          feeder_mode = FEEDER_LOADING;
        }
        else 
        {
          Serial.println("FEEDER: Drop Dwell ended.  Home to pause position.");
          feeder_homed = FEEDER_HOMING_TO_PAUSE;
          feeder_mode = FEEDER_HOMING;
        }
      }
      break;
    case FEEDER_DROPPING:
      if (isAtDropLocation()) 
      {
        Serial.print("FEEDER: at drop position: ");
        Serial.println(stepper.getCurrentPosition());
        stopStepper(true);
        Serial.println("FEEDER: Starting Drop Dwell.");
        feeder_dwell_start_time = now;
        feeder_mode = FEEDER_DWELL;
      }
      break;
    case FEEDER_LOADING:
      if (isAtPauseLocation())
      {
        stopStepper(false);
        Serial.print("FEEDER: at pause position: ");
        Serial.println(stepper.getCurrentPosition());
        Serial.println("FEEDER: Paused for next cycle.");
        feeder_mode = FEEDER_PAUSED;
      }
      break;
    default:
      if (Serial && DEBUG) Serial.println("ERROR: checkStepper(): Unknown steper mode");
      break;
  }
}

/* Start the stepper motor moving. */
void startStepper()
{
  if (!(tic_e)) return;
  if (Serial && DEBUG) Serial.println("startStepper()");
  if (stepper.getOperationState() != TicOperationState::Normal) stepper.exitSafeStart();
  if (!(stepper.getEnergized())) stepper.energize();
  stepper.setTargetVelocity(motor_speed * TIC_PULSE_MULTIPLIER);
  long mark = millis();
  while (stepper.getCurrentVelocity() == 0)
  {
    if (millis() - mark >= START_STOP_TIMEOUT)
    {
      if (Serial) Serial.println("ERROR: startStepper(): Timeout waiting on stepper to move.");
      //TODO: set error state
    }
  }
}

/* Stop the stepper motor.  Reset home position if specified. */
void stopStepper(bool setZero)
{
  if (!(tic_e)) return;
  if (Serial && DEBUG) Serial.print("stopStepper()"); Serial.println(setZero);
  stepper.setTargetVelocity(0);
  long mark = millis();
  while (stepper.getCurrentVelocity() > 0)
  {
    if (millis() - mark >= START_STOP_TIMEOUT)
    {
      if (Serial) Serial.println("ERROR: stopStepper(): Timeout waiting on stepper to stop.");
      //TODO: set error state
    }
  }
  if (setZero) stepper.haltAndSetPosition(0);
}

/* Refresh UI */
void refreshUI(unsigned long now)
{
  if ((unsigned long)(now - last_data_refresh) > data_refresh_interval)
  {
    last_data_refresh = now;
    char buf[_UI_TEMPORARY_STRING_BUFFER_SIZE];
    lv_color_t bg_color = lv_color_hex(0x0078EC);

    if (state != last_state)
    {

      if (last_state == SYS_HOMING)  //special cases for system exiting homing, setup buttons for annealing. 
      {
          lv_obj_clear_flag(ui_BtnAuto, LV_OBJ_FLAG_HIDDEN);
          lv_obj_clear_flag(ui_BtnManual, LV_OBJ_FLAG_HIDDEN);
          lv_obj_clear_flag(ui_BtnReset, LV_OBJ_FLAG_HIDDEN);
          lv_obj_set_style_bg_color(ui_BtnManual, lv_color_hex(0x0042c3), LV_PART_MAIN | LV_STATE_DEFAULT );
          lv_label_set_text(ui_BtnManualLabel, "Manual");
      }
          
      lv_obj_add_state(ui_BtnSettings, LV_STATE_DISABLED);  // disallow settings edit during running states, will clear below on non-running
      lv_obj_set_style_bg_color(ui_Temp, lv_color_hex(0x006e00), LV_PART_MAIN | LV_STATE_DEFAULT );  // default temp label color, will override if in cooling state below

      switch (state)
      {
        case SYS_NOT_HOMED:
          snprintf(buf, sizeof(buf), "%s", "Not Homed");
          bg_color = lv_color_hex(0xb300c1);
          lv_obj_add_flag(ui_BtnAuto, LV_OBJ_FLAG_HIDDEN);
          lv_obj_add_flag(ui_BtnReset, LV_OBJ_FLAG_HIDDEN);
          lv_obj_set_style_bg_color(ui_BtnManual, lv_color_hex(0x680185), LV_PART_MAIN | LV_STATE_DEFAULT );
          lv_label_set_text(ui_BtnManualLabel, "Home");
          lv_obj_clear_state(ui_BtnSettings, LV_STATE_DISABLED);
          break;
        case SYS_HOMING:
          snprintf(buf, sizeof(buf), "%s", "Homing");
          bg_color = lv_color_hex(0xb300c1);
          lv_obj_add_flag(ui_BtnManual, LV_OBJ_FLAG_HIDDEN);
          break;
        case SYS_MANUAL:
          snprintf(buf, sizeof(buf), "%s", "Manual");
          lv_obj_clear_state(ui_BtnSettings, LV_STATE_DISABLED);
          break;
        case SYS_PAUSED:
          snprintf(buf, sizeof(buf), "%s", "Paused");
          bg_color = lv_color_hex(0x006e00);
          lv_obj_clear_state(ui_BtnSettings, LV_STATE_DISABLED);
          break;
        case SYS_FEED_CASE:
          snprintf(buf, sizeof(buf), "%s", "Feed Case");
          bg_color = lv_color_hex(0x006e00);
          break;
        case SYS_ANNEALING:
          snprintf(buf, sizeof(buf), "%s", "Annealing");
          bg_color = lv_color_hex(0xff7900);
          break;
        case SYS_CASE_DROP:
          snprintf(buf, sizeof(buf), "%s", "Dropping Case");
          bg_color = lv_color_hex(0x006e00);
          break;
        case SYS_LOADING:
          snprintf(buf, sizeof(buf), "%s", "Loading Case");
          bg_color = lv_color_hex(0x0078EC);
          break;
        case SYS_COOLING:
          snprintf(buf, sizeof(buf), "%s", "Cooling Down");
          bg_color = lv_color_hex(0xdf0000);
          lv_obj_set_style_bg_color(ui_Temp, lv_color_hex(0xdf0000), LV_PART_MAIN | LV_STATE_DEFAULT );
          break;
        case SYS_ERROR:
          snprintf(buf, sizeof(buf), "%s", "ERROR!");
          bg_color = lv_color_hex(0xdf0000);
          break;
        default:
          if (Serial) 
          {
            Serial.print("ERROR: refreshUI(): unhandled system state: ");
            Serial.println(state);
          }
          state = SYS_ERROR;
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
    snprintf(buf, sizeof(buf), "%03.1f", sys_temp_F);
    lv_label_set_text(ui_Temp, buf);
  }
}

/* Read settings from storage.  Update globals as needed.
  Note: If EEPROM version is out of date, will set everything to defaults and write to storage.
*/
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
    feeder_dwell_time = prefs.storeData.feeder_dwell_time;
    drop_pos_offset = prefs.storeData.drop_pos_offset;
    motor_speed = prefs.storeData.motor_speed;
    motor_current = prefs.storeData.motor_current;
    temp_high_threshold = prefs.storeData.temp_high_threshold;
    temp_low_threshold = prefs.storeData.temp_low_threshold;
    if (Serial && DEBUG) Serial.println("Sys data updated from EEPROM stored preferences.");
    if (Serial && DEBUG)
    {
      Serial.print("   anneal_time: ");
      Serial.println(anneal_time);
      Serial.print("   pause_time: ");
      Serial.println(pause_time);
      Serial.print("   feeder_dwell_time: ");
      Serial.println(feeder_dwell_time);
      Serial.print("   drop_pos_offset: ");
      Serial.println(drop_pos_offset);
      Serial.print("   motor_speed: ");
      Serial.println(motor_speed);
      Serial.print("   motor_current: ");
      Serial.println(motor_current);
      Serial.print("   high_temp_threshold: ");
      Serial.println(temp_high_threshold);
      Serial.print("   low_temp_threshold: ");
      Serial.println(temp_low_threshold);
    }
    tmp.setHighTempF(temp_high_threshold);
    tmp.setLowTempF(temp_low_threshold);
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

/* Write settings to storage. */
void writeStorage()
{
  if (Serial && DEBUG) Serial.println("writeStorage()");
  EEPROM.begin(1024);
  prefs.storeData.version = EEPROM_VERSION;
  prefs.storeData.anneal_time = anneal_time;
  prefs.storeData.pause_time = pause_time;
  prefs.storeData.feeder_dwell_time = feeder_dwell_time;
  prefs.storeData.drop_pos_offset = drop_pos_offset;
  prefs.storeData.motor_speed = motor_speed;
  prefs.storeData.motor_current = motor_current;
  prefs.storeData.temp_high_threshold = temp_high_threshold;
  prefs.storeData.temp_low_threshold = temp_low_threshold;
  for(int i=0; i<STORED_DATA_SIZE; i++){
    EEPROM.writeByte(EEPROM_BASE+i, prefs.bufferData[i]);
  }
  EEPROM.end();
  if (Serial && DEBUG) Serial.println("Sys data written to EEPROM stored preferences.");
  if (Serial && DEBUG)
  {
    Serial.print("   anneal_time: ");
    Serial.println(anneal_time);
    Serial.print("   pause_time: ");
    Serial.println(pause_time);
    Serial.print("   feeder_dwell_time: ");
    Serial.println(feeder_dwell_time);
    Serial.print("   drop_pos_offset: ");
    Serial.println(drop_pos_offset);
    Serial.print("   motor_speed: ");
    Serial.println(motor_speed);
    Serial.print("   motor_current: ");
    Serial.println(motor_current);
    Serial.print("   high_temp_threshold: ");
    Serial.println(temp_high_threshold);
    Serial.print("   low_temp_threshold: ");
    Serial.println(temp_low_threshold);
  }
  tmp.setHighTempF(temp_high_threshold);
  tmp.setLowTempF(temp_low_threshold);
}
