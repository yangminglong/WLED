#pragma once


#include "wled.h"
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

}


void IRAM_ATTR onIndexCntInterrupt() {

}

class StepperCurtainUsermod : public Usermod {

  private:
    TMC2209 stepper_driver;

    bool initDone = false;
    bool enabled = true;

    int8_t pinEn   = PIN_LOAD_EN;
    int8_t pinDir  = PIN_DIR;
    int8_t pinStp  = PIN_STEP;
    int8_t pinTx   = PIN_SERIAL_TX;
    int8_t pinRx   = PIN_SERIAL_RX;
    int8_t pinIdx  = PIN_INDEX;    
    int8_t pinBtUp = PIN_BT_UP;
    int8_t pinBtDn = PIN_BT_DOWN;    
    int8_t pinEnd  = PIN_ENDSTOP;

    enum MoveState {
      Ready,
      MoveUp,
      MoveDown,
    };

    MoveState moveState = Ready;

    int64_t maxPos = 0;
    int64_t tarPos = 0;
    int64_t curPos = 0;

    static const char _name[];
    static const char _enabled[];
    static const char _tachoPin[];
    static const char _pwmPin[];
    static const char _temperature[];
    static const char _tachoUpdateSec[];
    static const char _minPWMValuePct[];
    static const char _maxPWMValuePct[];
    static const char _IRQperRotation[];
    static const char _speed[];
    static const char _lock[];



    void initStepper() {

    }

    void initButtons() {

    }
    
    void initEndstop() {
      if (pinEnd < 0 || !pinManager.allocatePin(pinEnd, false, PinOwner::UM_Unspecified)){
        pinEnd = -1;
        return;
      }
      pinMode(pinEnd, INPUT);
      digitalWrite(pinEnd, HIGH);
      attachInterrupt(digitalPinToInterrupt(pinEnd), onEndstopInterrupt, FALLING);
    }

    void deinitEndstop(void) {
      if (pinEnd < 0) return;
      detachInterrupt(digitalPinToInterrupt(pinEnd));
      pinManager.deallocatePin(pinEnd, PinOwner::UM_Unspecified);
      pinEnd = -1;
    }

  public:

    // gets called once at boot. Do all initialization that doesn't depend on
    // network here
    void setup() {
      initStepper();
      initButtons();
      initEndstop();
      initDone = true;
    }

    // gets called every time WiFi is (re-)connected. Initialize own network
    // interfaces here
    void connected() {}

    /*
     * Da loop.
     */
    void loop() {
      if (!enabled || strip.isUpdating()) return;

      if (moveState == MoveDown ) {
        if (curPos >= tarPos || (maxPos > 0 && curPos >= maxPos)) {
          stepper_driver.moveAtVelocity(0);
        }
      }
    }

    /*
     * addToJsonInfo() can be used to add custom entries to the /json/info part of the JSON API.
     * Creating an "u" object allows you to add custom key/value pairs to the Info section of the WLED web UI.
     * Below it is shown how this could be used for e.g. a light sensor
     */
    void addToJsonInfo(JsonObject& root) {
      JsonObject user = root["u"];
      if (user.isNull()) user = root.createNestedObject("u");

      JsonArray infoArr = user.createNestedArray(FPSTR(_name));
      String uiDomString = F("<button class=\"btn btn-xs\" onclick=\"requestJson({'");
      uiDomString += FPSTR(_name);
      uiDomString += F("':{'");
      uiDomString += FPSTR(_enabled);
      uiDomString += F("':");
      uiDomString += enabled ? "false" : "true";
      uiDomString += F("}});\"><i class=\"icons ");
      uiDomString += enabled ? "on" : "off";
      uiDomString += F("\">&#xe08f;</i></button>");
      infoArr.add(uiDomString);

      if (enabled) {
        JsonArray infoArr = user.createNestedArray(F("Manual"));
        String uiDomString = F("<div class=\"slider\"><div class=\"sliderwrap il\"><input class=\"noslide\" onchange=\"requestJson({'");
        uiDomString += FPSTR(_name);
        uiDomString += F("':{'");
        uiDomString += FPSTR(_speed);
        uiDomString += F("':parseInt(this.value)}});\" oninput=\"updateTrail(this);\" max=100 min=0 type=\"range\" value=");
        uiDomString += pwmValuePct;
        uiDomString += F(" /><div class=\"sliderdisplay\"></div></div></div>"); //<output class=\"sliderbubble\"></output>
        infoArr.add(uiDomString);

        JsonArray data = user.createNestedArray(F("Speed"));
        if (tachoPin >= 0) {
          data.add(last_rpm);
          data.add(F("rpm"));
        } else {
          if (lockFan) data.add(F("locked"));
          else         data.add(F("auto"));
        }
      }
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
      if (!initDone) return;  // prevent crash on boot applyPreset()
      JsonObject usermod = root[FPSTR(_name)];
      if (!usermod.isNull()) {
        if (usermod[FPSTR(_enabled)].is<bool>()) {
          enabled = usermod[FPSTR(_enabled)].as<bool>();
          if (!enabled) updateFanSpeed(0);
        }
        if (enabled && !usermod[FPSTR(_speed)].isNull() && usermod[FPSTR(_speed)].is<int>()) {
          pwmValuePct = usermod[FPSTR(_speed)].as<int>();
          updateFanSpeed((constrain(pwmValuePct,0,100) * 255) / 100);
          if (pwmValuePct) lockFan = true;
        }
        if (enabled && !usermod[FPSTR(_lock)].isNull() && usermod[FPSTR(_lock)].is<bool>()) {
          lockFan = usermod[FPSTR(_lock)].as<bool>();
        }
      }
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
      top[FPSTR(_enabled)]        = enabled;
      top[FPSTR(_pwmPin)]         = pwmPin;
      top[FPSTR(_tachoPin)]       = tachoPin;
      top[FPSTR(_tachoUpdateSec)] = tachoUpdateSec;
      top[FPSTR(_temperature)]    = targetTemperature;
      top[FPSTR(_minPWMValuePct)] = minPWMValuePct;
      top[FPSTR(_maxPWMValuePct)] = maxPWMValuePct;
      top[FPSTR(_IRQperRotation)] = numberOfInterrupsInOneSingleRotation;
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
      int8_t newTachoPin = tachoPin;
      int8_t newPwmPin   = pwmPin;

      JsonObject top = root[FPSTR(_name)];
      DEBUG_PRINT(FPSTR(_name));
      if (top.isNull()) {
        DEBUG_PRINTLN(F(": No config found. (Using defaults.)"));
        return false;
      }

      enabled           = top[FPSTR(_enabled)] | enabled;
      newTachoPin       = top[FPSTR(_tachoPin)] | newTachoPin;
      newPwmPin         = top[FPSTR(_pwmPin)] | newPwmPin;
      tachoUpdateSec    = top[FPSTR(_tachoUpdateSec)] | tachoUpdateSec;
      tachoUpdateSec    = (uint8_t) max(1,(int)tachoUpdateSec); // bounds checking
      targetTemperature = top[FPSTR(_temperature)] | targetTemperature;
      minPWMValuePct    = top[FPSTR(_minPWMValuePct)] | minPWMValuePct;
      minPWMValuePct    = (uint8_t) min(100,max(0,(int)minPWMValuePct)); // bounds checking
      maxPWMValuePct    = top[FPSTR(_maxPWMValuePct)] | maxPWMValuePct;
      maxPWMValuePct    = (uint8_t) min(100,max((int)minPWMValuePct,(int)maxPWMValuePct)); // bounds checking
      numberOfInterrupsInOneSingleRotation = top[FPSTR(_IRQperRotation)] | numberOfInterrupsInOneSingleRotation;
      numberOfInterrupsInOneSingleRotation = (uint8_t) max(1,(int)numberOfInterrupsInOneSingleRotation); // bounds checking

      if (!initDone) {
        // first run: reading from cfg.json
        tachoPin = newTachoPin;
        pwmPin   = newPwmPin;
        DEBUG_PRINTLN(F(" config loaded."));
      } else {
        DEBUG_PRINTLN(F(" config (re)loaded."));
        // changing paramters from settings page
        if (tachoPin != newTachoPin || pwmPin != newPwmPin) {
          DEBUG_PRINTLN(F("Re-init pins."));
          // deallocate pin and release interrupts
          deinitTacho();
          deinitPWMfan();
          tachoPin = newTachoPin;
          pwmPin   = newPwmPin;
          // initialise
          setup();
        }
      }

      // use "return !top["newestParameter"].isNull();" when updating Usermod with new features
      return !top[FPSTR(_IRQperRotation)].isNull();
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
const char StepperCurtainUsermod::_name[]           PROGMEM = "PWM-fan";
const char StepperCurtainUsermod::_enabled[]        PROGMEM = "enabled";
const char StepperCurtainUsermod::_tachoPin[]       PROGMEM = "tacho-pin";
const char StepperCurtainUsermod::_pwmPin[]         PROGMEM = "PWM-pin";
const char StepperCurtainUsermod::_temperature[]    PROGMEM = "target-temp-C";
const char StepperCurtainUsermod::_tachoUpdateSec[] PROGMEM = "tacho-update-s";
const char StepperCurtainUsermod::_minPWMValuePct[] PROGMEM = "min-PWM-percent";
const char StepperCurtainUsermod::_maxPWMValuePct[] PROGMEM = "max-PWM-percent";
const char StepperCurtainUsermod::_IRQperRotation[] PROGMEM = "IRQs-per-rotation";
const char StepperCurtainUsermod::_speed[]          PROGMEM = "speed";
const char StepperCurtainUsermod::_lock[]           PROGMEM = "lock";
