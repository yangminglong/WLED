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
          DEBUG_PRINTLN("PWM output is already open.");
          return;  // PWM output is already open
        } else {
          close();  // Config has changed, close and reopen
        }
      }

      pin_ = pin;
      freq_ = freq;
      if (pin_ < 0) {
        DEBUG_PRINTLN("PWM output pin is not valid.");
        return;
      }

      DEBUG_PRINTF("pwm_output[%d]: setup to freq %d\n", pin_, freq_);
      if (!pinManager.allocatePin(pin_, true, PinOwner::UM_PWM_OUTPUTS)) {
        DEBUG_PRINTLN("PWM output allocatePin failed.");
        return;
      }
      
      channel_ = pinManager.allocateLedc(1);
      if (channel_ == 255) {
        DEBUG_PRINTF("pwm_output[%d]: failed to quire ledc\n", pin_);
        pinManager.deallocatePin(pin_, PinOwner::UM_PWM_OUTPUTS);
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
      pinManager.deallocatePin(pin_, PinOwner::UM_PWM_OUTPUTS);
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
      char buffer[16] = {0};
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
      if (pwmConfig.isNull()) {
        DEBUG_PRINTLN("PWM readFromConfig failed, pwm config isNull.");
        return false;
      }
        
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
  // ServoFeed()
  // {
  //   // for (int i=0; i < USERMOD_FEED_TIMES; i++) 
  //   // {
  //   //   feedInfos[i].feed = this;
  //   // }
  // }

  bool isEnabled() const {
    return pwm.isEnabled();
  }

  struct FeedTickerInfo {
    Ticker ticker;
    ServoFeed* feed;
  };

  void feed() {
    DEBUG_PRINTF("feed now:%.3f.\n", servoFeedPos);

    pwm.setDuty(servoFeedPos);

    moveTicker.once_ms<ServoFeed*>(feedbackDelay, [](ServoFeed* feed) {
      DEBUG_PRINTF("move to standby %.3f.\n", feed->servoStandbyPos);
      feed->pwm.setDuty(feed->servoStandbyPos);
    }, this);
  }

  time_t getCurSecsOfDay() const {
    updateLocalTime();
    uint8_t hr = hour(localTime);
    uint8_t mi = minute(localTime);
    uint8_t se = second(localTime);

    DEBUG_PRINTF("updateFeedTicker: curr time %d:%d:%d. \n", hr, mi, se);

    return hr*SECS_PER_HOUR + mi*SECS_PER_MIN + se;
  }

  void updateFeedTicker() {
    DEBUG_PRINTLN(F("updateFeedTicker now."));
    if (!isEnabled())
      return;

    time_t currSecofDay = getCurSecsOfDay();

    for (int i=0; i < USERMOD_FEED_TIMES; i++) 
    {

      time_t feedTime = feedTimes[i];
      FeedTickerInfo& feedInfo = feedInfos[i];
      if (feedTime < 0) {
        DEBUG_PRINTLN(F("updateFeedTicker, ticker detach."));
        feedInfo.ticker.detach();
        return;
      }

      time_t remainder =  feedTime - currSecofDay;
      if (remainder <=0) 
        remainder = remainder + SECS_PER_DAY; 

      feedInfo.feed = this;
      
      DEBUG_PRINTF("updateFeedTicker: next feed at %ds later. \n", remainder);

      feedInfo.ticker.once<FeedTickerInfo*>(remainder, [](FeedTickerInfo* feedInfo)
      {
        feedInfo->feed->feed();
        feedInfo->ticker.attach<ServoFeed*>(SECS_PER_DAY, [](ServoFeed* feed)
        {
           feed->feed(); 
        }, feedInfo->feed);
      }, &feedInfo);
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
      String key = String(i)+"FeedTime:";
      state[key] = fmtTime;
    }

    pwm.addToJsonState(state);
  }

  void readFromJsonState(JsonObject& state) {
    if (state.isNull()) {
      return;
    }

    bool feedNow = false;
    if (getJsonValue(state[F("feedNow")], feedNow, false) && feedNow) {
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
    user[F("NextFeedTime:"   )] = fmtTime;    
  }



  void addToConfig(JsonObject& servoFeedConfig) const {
    servoFeedConfig[F("standbyPos"   )] = String(servoStandbyPos) ;
    servoFeedConfig[F("feedPos"      )] = String(servoFeedPos)   ;
    servoFeedConfig[F("feedbackDelay")] = feedbackDelay  ;

    for(int i=0;i < USERMOD_FEED_TIMES; ++i) {
      char buffer[16] = {0};
      sprintf_P(buffer, PSTR("feedTime %d"), i);

      uint8_t hr = hour(feedTimes[i]);
      uint8_t mi = minute(feedTimes[i]);
      
      servoFeedConfig[buffer] = String(hr) + "." + String(mi);
    }

    pwm.addToConfig(servoFeedConfig);
  }



  time_t stringToTime(const String& strTime) {
    int idx = strTime.indexOf(".");
    int hh = strTime.substring(0, idx).toInt();
    int mm = strTime.substring(idx+1).toInt();
    hh = constrain(hh, 0, 23);
    mm = constrain(mm, 0, 59);
    time_t t;
    t = hh*SECS_PER_HOUR + mm*SECS_PER_MIN;
    DEBUG_PRINTF("stringToTime: %s-> %d:%d -> %d \n", strTime, hh, mm, t);
    return t;
  }


  bool readFromConfig(JsonObject& servoFeedConfig) {
    if (servoFeedConfig.isNull())
      return false;
      
    bool configComplete = true;

    String strPos0, strPos1;

    configComplete &= getJsonValue(servoFeedConfig[F("standbyPos"   )], strPos0, "0.04");  
    configComplete &= getJsonValue(servoFeedConfig[F("feedPos"      )], strPos1, "0.12");
    configComplete &= getJsonValue(servoFeedConfig[F("feedbackDelay")], feedbackDelay  , 2000);

    servoStandbyPos = strPos0.toFloat();
    servoFeedPos = strPos1.toFloat();

    for(int i=0;i < USERMOD_FEED_TIMES; ++i) {
      char buffer[16] = {0};
      sprintf_P(buffer, PSTR("feedTime %d"), i);
      String feedTime;
      configComplete &= getJsonValue(servoFeedConfig[buffer], feedTime, "0.0");
      feedTimes[i] = stringToTime(feedTime);
    }

    feedbackDelay = constrain(feedbackDelay, 1000, 1000*60);
    DEBUG_PRINTF("feedbackDelay: %d \n", feedbackDelay);

    pwm.readFromConfig(servoFeedConfig);

    pwm.setDuty(servoFeedPos);

    moveTicker.once_ms<ServoFeed*>(feedbackDelay, [](ServoFeed* feed) {
      feed->pwm.setDuty(feed->servoStandbyPos);
      feed->updateFeedTicker();
    }, this);

    return configComplete;
  }

  
private:
  float servoStandbyPos = 0.08;
  float servoFeedPos = 0.12;
  uint32_t feedbackDelay = 2000;
  time_t feedTimes[USERMOD_FEED_TIMES];  // second on day
  FeedTickerInfo feedInfos[USERMOD_FEED_TIMES];
  Ticker moveTicker;

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
      DEBUG_PRINTLN(F("usermod servo_feed addToJsonState."));
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
      DEBUG_PRINTLN(F("usermod servo_feed readFromJsonState."));
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
      DEBUG_PRINTLN(F("usermod servo_feed addToJsonInfo."));
      JsonObject user = root[F("u")];
      if (user.isNull())
        user = root.createNestedObject(F("u"));

      for (int i = 0; i < USERMOD_PWM_SERVO_PINS; i++) {
        const ServoFeed& feed = feeds[i];
        feed.addToJsonInfo(user);
      }
    }

    void addToConfig(JsonObject& root) {
      DEBUG_PRINTLN(F("usermod servo_feed addToConfig."));
      JsonObject top = root.createNestedObject(USERMOD_NAME);
      for (int i = 0; i < USERMOD_PWM_SERVO_PINS; i++) {
        const ServoFeed& feed = feeds[i];
        char buffer[16]= {0};
        sprintf_P(buffer, PSTR("Feed %d"), i);
        JsonObject feedConfig = top.createNestedObject(buffer);
        feed.addToConfig(feedConfig);
      }
    }

    bool readFromConfig(JsonObject& root) {
      DEBUG_PRINTLN(F("usermod servo_feed readFromConfig."));

      JsonObject top = root[USERMOD_NAME];
      if (top.isNull())
        return false;

      bool configComplete = true;
      for (int i = 0; i < USERMOD_PWM_SERVO_PINS; i++) {
        ServoFeed& feed = feeds[i];
        char buffer[16] = {0};
        sprintf_P(buffer, PSTR("Feed %d"), i);
        JsonObject feedConfig = top[buffer];
        configComplete &= feed.readFromConfig(feedConfig);
      }
      return configComplete;
    }

    uint16_t getId() {
      MISO;
      return USERMOD_ID_SERVO_FEED;
    }

  private:
    ServoFeed feeds[USERMOD_PWM_SERVO_PINS];
};

const char ServoFeedUsermod::USERMOD_NAME[] PROGMEM = "ServoFeed";
const char ServoFeedUsermod::STATE_NAME[] PROGMEM = "ServoFeedState";
