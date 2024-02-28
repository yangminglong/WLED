#pragma once
#include "idf_iot_button.h"
#define CONFIG_BUTTON_PERIOD_TIME_MS 5
#define CONFIG_BUTTON_DEBOUNCE_TICKS 2
#define CONFIG_BUTTON_SHORT_PRESS_TIME_MS 180
#define CONFIG_BUTTON_LONG_PRESS_TIME_MS 1500
#define CONFIG_BUTTON_LONG_PRESS_TOLERANCE_MS 20
#define CONFIG_BUTTON_SERIAL_TIME_MS 20
#define CONFIG_ADC_BUTTON_MAX_CHANNEL 3
#define CONFIG_ADC_BUTTON_MAX_BUTTON_PER_CHANNEL 8
#define CONFIG_ADC_BUTTON_SAMPLE_TIMES 1


#include "wled.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>

#define PIN_LOAD_EN   0 
#define PIN_DIR       0     
#define PIN_STEP      0
#define PIN_SERIAL_TX 0     
#define PIN_SERIAL_RX 0     
#define PIN_INDEX     0
#define PIN_BT_UP     0  
#define PIN_BT_DOWN   0  
#define PIN_ENDSTOP   0

#include <TMC2209.h>


void IRAM_ATTR onEndstopInterrupt() {
  StepperCurtainUsermod* mod = (StepperCurtainUsermod*)usermods.lookup(USERMOD_ID_STEPPER_CURTAIN);
  mod->stopMove(true);
}

void IRAM_ATTR onIndexCntInterrupt() {
  StepperCurtainUsermod* mod = (StepperCurtainUsermod*)usermods.lookup(USERMOD_ID_STEPPER_CURTAIN);
  mod->updateCnt();
}

class StepperCurtainUsermod : public Usermod {

private:
  // Define a mutex
  SemaphoreHandle_t xSemaphore;
  TMC2209 stepper_driver;

  enum MoveState {
    Ready,
    MoveDown, // positive
    MoveUp,   // negative
  };

  MoveState moveState = Ready;

  bool initDone = false;

  int64_t tarPos = 0;
  int64_t cntIdx = 0;

  int8_t pinEn   = PIN_LOAD_EN;    // Output
  int8_t pinDir  = PIN_DIR;        // Output
  int8_t pinStp  = PIN_STEP;       // Output
  int8_t pinTx   = PIN_SERIAL_TX;  // Output
  int8_t pinRx   = PIN_SERIAL_RX;  // Input
  int8_t pinIdx  = PIN_INDEX;      // Input
  int8_t pinBtDn = PIN_BT_DOWN;    
  int8_t pinBtUp = PIN_BT_UP;
  int8_t pinEnd  = PIN_ENDSTOP;

  int64_t maxPos = 0;

  // stepper attr
  int64_t speed = 20000;
  uint8_t cPercent = 100; // run current percent

  button_handle_t btnHandleDown = 0;
  button_handle_t btnHandleUp   = 0;

  button_config_t btnConfigDown ;
  button_config_t btnConfigUp   ;

  static const char _name    [];
  static const char _pinDir  [];
  static const char _pinStp  [];
  static const char _pinTx   [];
  static const char _pinRx   [];
  static const char _pinIdx  [];
  static const char _pinBtDn [];
  static const char _pinBtUp [];
  static const char _pinEnd  [];
  static const char _maxPos  [];
  static const char _speed   [];
  static const char _cPercent[];


  bool initStepper() {
    std::vector<int8_t*> hasAllocaPins;
    std::vector<int8_t*> pins = { &pinEn, &pinDir, &pinStp, &pinTx, &pinRx, &pinIdx };

    for (int i = 0; i < pins.size(); ++i) {
      int8_t& pin = *pins[i];
      if (pin >= 0 && pinManager.allocatePin(pin, true, PinOwner::UM_Unspecified)) {
        hasAllocaPins.push_back(&pin);
      } else {
        pin = -1;
        break;
      }
    }
    
    if (hasAllocaPins.size() != pins.size()) {
      for (int i = 0; i < hasAllocaPins.size(); ++i) {
       int8_t& pin = *hasAllocaPins[i];
       pinManager.deallocatePin(pin, PinOwner::UM_Unspecified);
       pin = -1;
      }
      return false;
    }

    pinMode(pinEn , OUTPUT);
    pinMode(pinDir, OUTPUT);
    pinMode(pinStp, OUTPUT);
    // pinMode(pinTx , OUTPUT);
    // pinMode(pinRx , INPUT);
    pinMode(pinIdx, INPUT);

    digitalWrite(pinEn , HIGH);
    digitalWrite(pinDir, LOW);
    digitalWrite(pinStp, LOW);

    attachInterrupt(digitalPinToInterrupt(pinIdx), onIndexCntInterrupt, RISING);

    Serial1.setPins(pinRx, pinTx);
    stepper_driver.setup(Serial1);
    //delay(100);
    stepper_driver.setRunCurrent(cPercent);
    stepper_driver.enableCoolStep();
    stepper_driver.enable();

    return true;
  }

  void deinitSteppers() {
    std::vector<int8_t*> pins = {&pinEn, &pinDir, &pinStp, &pinTx, &pinRx, &pinIdx};
    for (int i = 0; i < pins.size(); ++i) {
      int8_t& pin = *pins.at(i);
      if (pin >= 0 ) {
        pinManager.deallocatePin(pin, PinOwner::UM_Unspecified);
        pin = -1;
      } 
    }
  }

  bool initButtons() {
    if (pinBtDn < 0 || !pinManager.allocatePin(pinBtDn, false, PinOwner::UM_Unspecified)){
      pinBtDn = -1;
      return false;
    }
    if (pinBtUp < 0 || !pinManager.allocatePin(pinBtUp, false, PinOwner::UM_Unspecified)){
      pinManager.deallocatePin(pinBtDn, PinOwner::UM_Unspecified);
      pinBtUp = -1;
      return false;
    }

    ESP_LOGI(TAG, "setupButton.");
    btnConfigDown = {
        .type = BUTTON_TYPE_GPIO,
        .long_press_time = CONFIG_BUTTON_LONG_PRESS_TIME_MS,
        .short_press_time = CONFIG_BUTTON_SHORT_PRESS_TIME_MS,
        .gpio_button_config = {
            .gpio_num = pinBtDn,
            .active_level = 0,
        },
    };

    btnConfigUp = btnConfigDown;
    btnConfigUp.gpio_button_config.gpio_num = pinBtUp;

    btnHandleDown = iot_button_create(&btnConfigDown);
    iot_button_register_cb(btnHandleDown, BUTTON_DOUBLE_CLICK, [](void *button_handle, void *usr_data){
      StepperCurtainUsermod* mod = (StepperCurtainUsermod*)usermods.lookup(USERMOD_ID_STEPPER_CURTAIN);
      mod->moveToMaxPos();
    }, NULL);
    iot_button_register_cb(btnHandleDown, BUTTON_PRESS_DOWN, [](void *button_handle, void *usr_data){
      StepperCurtainUsermod* mod = (StepperCurtainUsermod*)usermods.lookup(USERMOD_ID_STEPPER_CURTAIN);
      mod->moveDown();
    }, NULL);
    iot_button_register_cb(btnHandleDown, BUTTON_PRESS_UP, [](void *button_handle, void *usr_data){
      StepperCurtainUsermod* mod = (StepperCurtainUsermod*)usermods.lookup(USERMOD_ID_STEPPER_CURTAIN);
      mod->stopMove();
    }, NULL);

    btnHandleUp = iot_button_create(&btnConfigUp);
    iot_button_register_cb(btnHandleUp, BUTTON_DOUBLE_CLICK, [](void *button_handle, void *usr_data){
      StepperCurtainUsermod* mod = (StepperCurtainUsermod*)usermods.lookup(USERMOD_ID_STEPPER_CURTAIN);
      mod->moveToHomePos();
    }, NULL);
    iot_button_register_cb(btnHandleUp, BUTTON_PRESS_DOWN, [](void *button_handle, void *usr_data){
      StepperCurtainUsermod* mod = (StepperCurtainUsermod*)usermods.lookup(USERMOD_ID_STEPPER_CURTAIN);
      mod->moveUp();
    }, NULL);
    iot_button_register_cb(btnHandleUp, BUTTON_PRESS_UP, [](void *button_handle, void *usr_data){
      StepperCurtainUsermod* mod = (StepperCurtainUsermod*)usermods.lookup(USERMOD_ID_STEPPER_CURTAIN);
      mod->stopMove();
    }, NULL);
  }

  void deinitButtons() {
    if (pinBtDn >= 0) {
      pinManager.deallocatePin(pinBtDn, PinOwner::UM_Unspecified);
      pinBtDn = -1;
    }
    if (pinBtUp >= 0) {
      pinManager.deallocatePin(pinBtUp, PinOwner::UM_Unspecified);
      pinBtUp = -1;
    }

    if (btnHandleDown) iot_button_delete(btnHandleDown);
    if (btnHandleUp  ) iot_button_delete(btnHandleUp  );
  }
  
  bool initEndstop() {
    if (pinEnd < 0 || !pinManager.allocatePin(pinEnd, false, PinOwner::UM_Unspecified)){
      pinEnd = -1;
      return false;
    }
    pinMode(pinEnd, INPUT);

    attachInterrupt(digitalPinToInterrupt(pinEnd), onEndstopInterrupt, FALLING);

    return true;
  }

  void deinitEndstop() {
    if (pinEnd >= 0)  {
      detachInterrupt(digitalPinToInterrupt(pinEnd));
      pinManager.deallocatePin(pinEnd, PinOwner::UM_Unspecified);
      pinEnd = -1;
    }
  }

public:
  void updateCnt()
  {
    if( xSemaphoreTake( xSemaphore, pdMS_TO_TICKS(50) ) == pdTRUE )
    {
      moveState == MoveUp ? --cntIdx : moveState == MoveDown ? ++cntIdx : cntIdx;
      xSemaphoreGive( xSemaphore );
    }
  }

  void clearCnt()
  {
    if( xSemaphoreTake( xSemaphore, pdMS_TO_TICKS(50) ) == pdTRUE )
    {
      cntIdx = 0;
      xSemaphoreGive( xSemaphore );
    }
  }
  void stopMove(bool isEndstop = false) {
    if( xSemaphoreTake( xSemaphore, pdMS_TO_TICKS(50) ) == pdTRUE )
    {
      if (isEndstop) {
        cntIdx = 0;
        if (moveState == MoveUp) {
          stepper_driver.moveAtVelocity(0);
          moveState = Ready;
        }
      } else {
        stepper_driver.moveAtVelocity(0);
        moveState = Ready;
      }
      xSemaphoreGive( xSemaphore );
    }
  }
  void moveToMaxPos() {
    if (maxPos)
      moveTo(maxPos);
  }
  void moveToHomePos() {
    if (maxPos)
      moveTo(0);
  }
  void moveUp() {
    moveTo(std::numeric_limits<int64_t>::min());
  }
  void moveDown() {
    moveTo(std::numeric_limits<int64_t>::max());
  }
  void moveTo(int64_t _tarPos) {
    if( xSemaphoreTake( xSemaphore, pdMS_TO_TICKS(50) ) == pdTRUE )
    {
      tarPos = _tarPos;
      moveState = tarPos > cntIdx ? MoveDown : tarPos < cntIdx ? MoveUp : moveState;
      if (moveState == MoveUp)
        stepper_driver.enableInverseMotorDirection();
      else
        stepper_driver.disableInverseMotorDirection();

      stepper_driver.moveAtVelocity(speed);

      xSemaphoreGive( xSemaphore );
    }
  }
  
public:

  // gets called once at boot. Do all initialization that doesn't depend on
  // network here
  void setup() {
    if (initStepper() && initButtons() && initEndstop() )
      initDone = true;
  }

  // gets called every time WiFi is (re-)connected. Initialize own network
  // interfaces here
  void connected() {}

  /*
    * Da loop.
    */
  void loop() {
    if (!initDone || strip.isUpdating()) return;

    switch (moveState)
    {
    case MoveDown:
      if (cntIdx >= tarPos || (maxPos > 0 && cntIdx >= maxPos)) {
        stopMove();
      }
      break;
    case MoveUp:
      if (cntIdx <= tarPos  || (maxPos > 0 && cntIdx <= 0)) {
        stopMove();
      }
      break;    
    default:
      break;
    }
  }

  /*
    * addToJsonInfo() can be used to add custom entries to the /json/info part of the JSON API.
    * Creating an "u" object allows you to add custom key/value pairs to the Info section of the WLED web UI.
    * Below it is shown how this could be used for e.g. a light sensor
    */
  void addToJsonInfo(JsonObject& root) {
  }

  /*
    * addToJsonState() can be used to add custom entries to the /json/state part of the JSON API (state object).
    * Values in the state object may be modified by connected clients
    */
  //void addToJsonState(JsonObject& root) {
  //}

  /*
    * readFromJsonState() can be used to receive data clients send to the /json/state part of the JSON API (state object).
    * Values in the state object may be modified by connected clients
    */
  void readFromJsonState(JsonObject& root) {

  }

  /*
    * addToConfig() can be used to add custom persistent settings to the cfg.json file in the "um" (usermod) object.
    * It will be called by WLED when settings are actually saved (for example, LED settings are saved)
    * If you want to force saving the current state, use serializeConfig() in your loop().
    * 
    * CAUTION: serializeConfig() will initiate a filesystem write operation.
    * It might cause the LEDs to stutter and will cause flash wear if called too often.
    * Use it sparingly and always in the loop, never in network callbacks!
    * 
    * addToConfig() will also not yet add your setting to one of the settings pages automatically.
    * To make that work you still have to add the setting to the HTML, xml.cpp and set.cpp manually.
    * 
    * I highly recommend checking out the basics of ArduinoJson serialization and deserialization in order to use custom settings!
    */
  void addToConfig(JsonObject& root) {
    JsonObject top = root.createNestedObject(FPSTR(_name)); // usermodname

    top[FPSTR(_pinDir  )]        = pinDir  ;
    top[FPSTR(_pinStp  )]        = pinStp  ;
    top[FPSTR(_pinTx   )]        = pinTx   ;
    top[FPSTR(_pinRx   )]        = pinRx   ;
    top[FPSTR(_pinIdx  )]        = pinIdx  ;
    top[FPSTR(_pinBtDn )]        = pinBtDn ;
    top[FPSTR(_pinBtUp )]        = pinBtUp ;
    top[FPSTR(_pinEnd  )]        = pinEnd  ;
    top[FPSTR(_maxPos  )]        = maxPos  ;
    top[FPSTR(_speed   )]        = speed   ;
    top[FPSTR(_cPercent)]        = cPercent;

    DEBUG_PRINTLN(F("Autosave config saved."));
  }

  /*
    * readFromConfig() can be used to read back the custom settings you added with addToConfig().
    * This is called by WLED when settings are loaded (currently this only happens once immediately after boot)
    * 
    * readFromConfig() is called BEFORE setup(). This means you can use your persistent values in setup() (e.g. pin assignments, buffer sizes),
    * but also that if you want to write persistent values to a dynamic buffer, you'd need to allocate it here instead of in setup.
    * If you don't know what that is, don't fret. It most likely doesn't affect your use case :)
    * 
    * The function should return true if configuration was successfully loaded or false if there was no configuration.
    */
  bool readFromConfig(JsonObject& root) {
    std::vector<int8_t*> pins = {&pinDir, &pinStp, &pinTx, &pinRx, &pinBtDn, &pinBtUp, &pinEnd};

    JsonObject top = root[FPSTR(_name)];
    DEBUG_PRINT(FPSTR(_name));
    if (top.isNull()) {
      DEBUG_PRINTLN(F(": No config found. (Using defaults.)"));
      return false;
    }

    int8_t  _pinDir   = top[FPSTR(_pinDir  )] | pinDir ;
    int8_t  _pinStp   = top[FPSTR(_pinStp  )] | pinStp ;
    int8_t  _pinTx    = top[FPSTR(_pinTx   )] | pinTx  ;
    int8_t  _pinRx    = top[FPSTR(_pinRx   )] | pinRx  ;
    int8_t  _pinIdx   = top[FPSTR(_pinIdx  )] | pinIdx ;
    int8_t  _pinBtDn  = top[FPSTR(_pinBtDn )] | pinBtDn;
    int8_t  _pinBtUp  = top[FPSTR(_pinBtUp )] | pinBtUp;
    int8_t  _pinEnd   = top[FPSTR(_pinEnd  )] | pinEnd ;
    maxPos   = top[FPSTR(_maxPos  )] | maxPos  ;
    speed    = top[FPSTR(_speed   )] | speed   ;
    cPercent = top[FPSTR(_cPercent)] | cPercent;

    if (initDone == false) {
      // first run: reading from cfg.json
      DEBUG_PRINTLN(F(" config loaded."));
    } else {
      DEBUG_PRINTLN(F(" config (re)loaded."));
      // changing paramters from settings page
      bool pinHasChanged = false;
      std::vector<int8_t> newPins = {_pinDir, _pinStp, _pinTx, _pinRx, _pinBtDn, _pinBtUp, _pinEnd};
      for (int i = 0; i < pins.size(); ++i) {
        if (newPins[i] != *pins[i]) {
          pinHasChanged = true;
          break;
        }
      }
      if (pinHasChanged) {
        DEBUG_PRINTLN(F("Re-init pins."));
        // deallocate pin and release interrupts
        deinitSteppers();
        deinitButtons();
        deinitEndstop();

        for (int i = 0; i < pins.size(); ++i) {
          *pins[i] = newPins[i];
        }
        // initialise
        setup();
      }
    }

    // use "return !top["newestParameter"].isNull();" when updating Usermod with new features
    return true;
}

  /*
    * getId() allows you to optionally give your V2 usermod an unique ID (please define it in const.h!).
    * This could be used in the future for the system to determine whether your usermod is installed.
    */
  uint16_t getId() {
      return USERMOD_ID_STEPPER_CURTAIN;
  }
};

// strings to reduce flash memory usage (used more than twice)
const char StepperCurtainUsermod::_name    [] PROGMEM = "TMC2209";
const char StepperCurtainUsermod::_pinDir  [] PROGMEM = "Pin Dir";
const char StepperCurtainUsermod::_pinStp  [] PROGMEM = "Pin Step";
const char StepperCurtainUsermod::_pinTx   [] PROGMEM = "Pin Tx";
const char StepperCurtainUsermod::_pinRx   [] PROGMEM = "Pin Rx";
const char StepperCurtainUsermod::_pinIdx  [] PROGMEM = "Pin Index";
const char StepperCurtainUsermod::_pinBtDn [] PROGMEM = "Pin Button Down";
const char StepperCurtainUsermod::_pinBtUp [] PROGMEM = "Pin Button Up";
const char StepperCurtainUsermod::_pinEnd  [] PROGMEM = "Pin Endstop ";
const char StepperCurtainUsermod::_maxPos  [] PROGMEM = "Limit Max Pos";
const char StepperCurtainUsermod::_speed   [] PROGMEM = "Speed";
const char StepperCurtainUsermod::_cPercent[] PROGMEM = "Current Percent";