#pragma once
#include "wled.h"
#include <Ticker.h>

#ifndef ESP32
  #error This usermod does not support the ESP8266.
#endif

#ifndef USERMOD_PWM_SERVO_PINS
  #define USERMOD_PWM_SERVO_PINS 1
#endif

#ifndef USERMOD_FEED_TIMES
  #define USERMOD_FEED_TIMES 2
#endif

class PwmOutput {
  public:

    void open(int8_t pin, uint32_t freq) {

      if (enabled_) {
        if (pin == pin_ && freq == freq_) {
          return;  // PWM output is already open
        } else {
          close();  // Config has changed, close and reopen
        }
      }

      pin_ = pin;
      freq_ = freq;
      if (pin_ < 0) 
        return;

      DEBUG_PRINTF("pwm_output[%d]: setup to freq %d\n", pin_, freq_);
      if (!pinManager.allocatePin(pin_, true, PinOwner::UM_SERVO_FEED))
        return;
      
      channel_ = pinManager.allocateLedc(1);
      if (channel_ == 255) {
        DEBUG_PRINTF("pwm_output[%d]: failed to quire ledc\n", pin_);
        pinManager.deallocatePin(pin_, PinOwner::UM_SERVO_FEED);
        return;
      }

      ledcSetup(channel_, freq_, bit_depth_);
      ledcAttachPin(pin_, channel_);
      DEBUG_PRINTF("pwm_output[%d]: init successful\n", pin_);
      enabled_ = true;
    }

    void close() {
      DEBUG_PRINTF("pwm_output[%d]: close\n", pin_);
      if (!enabled_)
        return;
      pinManager.deallocatePin(pin_, PinOwner::UM_SERVO_FEED);
      if (channel_ != 255)
        pinManager.deallocateLedc(channel_, 1);
      channel_ = 255;
      duty_ = 0.0f;
      enabled_ = false;
    }

    void setDuty(const float duty) {
      DEBUG_PRINTF("pwm_output[%d]: set duty %f\n", pin_, duty);
      if (!enabled_)
        return;
      duty_ = min(1.0f, max(0.0f, duty));
      const uint32_t value = static_cast<uint32_t>((1 << bit_depth_) * duty_);
      ledcWrite(channel_, value);
    }

    void setDuty(const uint16_t duty) {
      setDuty(static_cast<float>(duty) / 65535.0f);
    }

    bool isEnabled() const {
      return enabled_;
    }

    void addToJsonState(JsonObject& pwmState) const {
      pwmState[F("duty")] = duty_;
    }

    void readFromJsonState(JsonObject& pwmState) {
      if (pwmState.isNull()) {
        return;
      }
      float duty;
      if (getJsonValue(pwmState[F("duty")], duty)) {
        setDuty(duty);
      }
    }

    void addToJsonInfo(JsonObject& user) const {
      if (!enabled_)
        return;
      char buffer[12];
      sprintf_P(buffer, PSTR("PWM pin %d"), pin_);
      JsonArray data = user.createNestedArray(buffer);
      data.add(1e2f * duty_);
      data.add(F("%"));
    }

    void addToConfig(JsonObject& pwmConfig) const {
      pwmConfig[F("pin")] = pin_;
      pwmConfig[F("freq")] = freq_;
    }

    bool readFromConfig(JsonObject& pwmConfig) {
      if (pwmConfig.isNull())
        return false;
        
      bool configComplete = true;
      int8_t newPin = pin_;
      uint32_t newFreq = freq_;
      configComplete &= getJsonValue(pwmConfig[F("pin")], newPin);  
      configComplete &= getJsonValue(pwmConfig[F("freq")], newFreq);

      open(newPin, newFreq);

      return configComplete;
    }

  private:
    int8_t pin_ {-1};
    uint32_t freq_ {50};
    static const uint8_t bit_depth_ {12};
    uint8_t channel_ {255};
    float duty_ {0.0f};
    bool enabled_ {false};
};


class ServoFeed
{
public:
  bool isEnabled() const {
    return pwm.isEnabled();
  }

  void feed() {
    pwm.setDuty(servoFeedPos);
    Ticker* ticker = new Ticker();
    ticker->once_ms(feedbackDelay, [this, ticker](){
      pwm.setDuty(servoStandbyPos);
      delete ticker;
    });
  }

  time_t getCurSecsOfDay() const {
    updateLocalTime();
    uint8_t hr = hour(localTime);
    uint8_t mi = minute(localTime);
    uint8_t se = second(localTime);

    return hr*3600 + mi*60 + se;
  }

  void updateFeedTicker() {
    if (!isEnabled())
      return;

    time_t currSecofDay = getCurSecsOfDay();

    feedTicker.clear();
    for (int i=0; i < USERMOD_FEED_TIMES; i++) 
    {
      time_t feedTime = feedTimes[i];
      if (feedTime < 0)
        return;

      feedTicker.push_back(Ticker());
      Ticker* ticker = &feedTicker.back();

      time_t remainder =  feedTime - currSecofDay;
      if (remainder <=0) 
        remainder = remainder + SECS_PER_DAY; 

      ticker->once(remainder, [this, ticker](){
        feed();
        ticker->attach(SECS_PER_DAY, [this](){feed();});
      });
    }
  }

  String formatTime(time_t t) const {
      uint8_t hr = hour(t);
      uint8_t mi = minute(t);
      uint8_t se = second(t);

      return String(hr)+":"+String(mi)+":"+String(se);
  }

  void addToJsonState(JsonObject& state) const {
    for(int i=0;i < USERMOD_FEED_TIMES; ++i) 
    {
      String  fmtTime = formatTime(feedTimes[i]);
      String key = "feedTime " + String(i);
      state[key] = fmtTime;
    }

    pwm.addToJsonState(state);
  }

  void readFromJsonState(JsonObject& state) {
    if (state.isNull()) {
      return;
    }

    bool feedNow = false;
    if (getJsonValue(state[F("feedNow")], feedNow, false)) {
      feed();
    }
  }

  void addToJsonInfo(JsonObject& user) const {
    if (!isEnabled())
      return;

    time_t remainder = -1;
    time_t currSecofDay = getCurSecsOfDay();
    bool hasFeed = false;

    for(int i=0;i < USERMOD_FEED_TIMES; ++i) 
    {
      hasFeed = hasFeed | (feedTimes[i] >= 0);
      if (feedTimes[i] > currSecofDay) {
        remainder = feedTimes[i] - currSecofDay;
        break;
      }
    }

    String fmtTime = isEnabled() && hasFeed ? "tomorrow" : "no feed";
    if (isEnabled() && remainder > 0) {
        fmtTime = formatTime(remainder);
    }
    user[F("nextFeedTime:"   )] = fmtTime;    
  }


  void addToConfig(JsonObject& servoFeedConfig) const {
    servoFeedConfig[F("standbyPos"   )] = servoStandbyPos;
    servoFeedConfig[F("feedPos"      )] = servoFeedPos   ;
    servoFeedConfig[F("feedbackDelay")] = feedbackDelay  ;

    for(int i=0;i < USERMOD_FEED_TIMES; ++i) {
      char buffer[8];
      sprintf_P(buffer, PSTR("feedTime %d"), i);
      servoFeedConfig[buffer] = feedTimes[i];
    }

    pwm.addToConfig(servoFeedConfig);
  }

  bool readFromConfig(JsonObject& servoFeedConfig) {
    if (servoFeedConfig.isNull())
      return false;
      
    bool configComplete = true;

    configComplete &= getJsonValue(servoFeedConfig[F("standbyPos"   )], servoStandbyPos, 0);  
    configComplete &= getJsonValue(servoFeedConfig[F("feedPos"      )], servoFeedPos   , 1);
    configComplete &= getJsonValue(servoFeedConfig[F("feedbackDelay")], feedbackDelay  , 2000);

    for(int i=0;i < USERMOD_FEED_TIMES; ++i) {
      char buffer[8];
      sprintf_P(buffer, PSTR("feedTime %d"), i);
      configComplete &= getJsonValue(servoFeedConfig[buffer], feedTimes[i], -1);
    }

    feedbackDelay = std::min(1000, feedbackDelay);

    pwm.readFromConfig(servoFeedConfig);
    pwm.setDuty(servoStandbyPos);

    updateFeedTicker();

    return configComplete;
  }

  
private:
  float servoStandbyPos = 0;
  float servoFeedPos = 1;
  int feedbackDelay = 2000;
  time_t feedTimes[USERMOD_FEED_TIMES];  // second on day
  std::vector<Ticker> feedTicker;  // second on day

  PwmOutput pwm;
};

class ServoFeedUsermod : public Usermod {
  public:

    static const char USERMOD_NAME[];
    static const char STATE_NAME[];

    void setup() {
      // By default all PWM outputs are disabled, no setup do be done
    }

    void loop() {
    }

    void addToJsonState(JsonObject& root) {
      JsonObject feedStates = root.createNestedObject(STATE_NAME);
      for (int i = 0; i < USERMOD_PWM_SERVO_PINS; i++) {
        const ServoFeed& feed = feeds[i];
        if (!feed.isEnabled())
          continue;
        char buffer[4];
        sprintf_P(buffer, PSTR("%d"), i);
        JsonObject feedState = feedStates.createNestedObject(buffer);
        feed.addToJsonState(feedState);
      }
    }

    void readFromJsonState(JsonObject& root) {
      JsonObject feedStates = root[STATE_NAME];
      if (feedStates.isNull())
        return;

      for (int i = 0; i < USERMOD_PWM_SERVO_PINS; i++) {
        ServoFeed& feed = feeds[i];
        if (!feed.isEnabled())
          continue;
        char buffer[4];
        sprintf_P(buffer, PSTR("%d"), i);
        JsonObject feedState = feedStates[buffer];
        feed.readFromJsonState(feedState);
      }
    }

    void addToJsonInfo(JsonObject& root) {
      JsonObject user = root[F("u")];
      if (user.isNull())
        user = root.createNestedObject(F("u"));

      for (int i = 0; i < USERMOD_PWM_SERVO_PINS; i++) {
        const ServoFeed& feed = feeds[i];
        feed.addToJsonInfo(user);
      }
    }

    void addToConfig(JsonObject& root) {
      JsonObject top = root.createNestedObject(USERMOD_NAME);
      for (int i = 0; i < USERMOD_PWM_SERVO_PINS; i++) {
        const ServoFeed& feed = feeds[i];
        char buffer[8];
        sprintf_P(buffer, PSTR("Feed %d"), i);
        JsonObject feedConfig = top.createNestedObject(buffer);
        feed.addToConfig(feedConfig);
      }
    }

    bool readFromConfig(JsonObject& root) {
      JsonObject top = root[USERMOD_NAME];
      if (top.isNull())
        return false;

      bool configComplete = true;
      for (int i = 0; i < USERMOD_PWM_SERVO_PINS; i++) {
        ServoFeed& feed = feeds[i];
        char buffer[8];
        sprintf_P(buffer, PSTR("Feed %d"), i);
        JsonObject feedConfig = top[buffer];
        configComplete &= feed.readFromConfig(feedConfig);
      }
      return configComplete;
    }

    uint16_t getId() {
      return USERMOD_ID_SERVO_FEED;
    }

  private:
    ServoFeed feeds[USERMOD_PWM_SERVO_PINS];
};

const char PwmOutputsUsermod::USERMOD_NAME[] PROGMEM = "ServoFeed";
const char PwmOutputsUsermod::STATE_NAME[] PROGMEM = "ServoFeedState";
