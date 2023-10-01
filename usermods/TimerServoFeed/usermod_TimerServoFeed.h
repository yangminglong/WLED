#pragma once
#include "wled.h"
#include "esp_timer.h"

#ifndef ESP32
  #error This usermod does not support the ESP8266.
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
      if (!pinManager.allocatePin(pin_, true, PinOwner::UM_PWM_OUTPUTS))
        return;
      
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
      // duty_ = 0.0f; // duty hold
      enabled_ = false;
    }

    void setDuty(const float duty) {
      DEBUG_PRINTF("pwm_output[%d]: set duty %f\n", pin_, duty);
      duty_ = min(1.0f, max(0.0f, duty));
      if (!enabled_)
        return;
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

    bool readFromConfig(JsonObject& pwmConfig, bool en) {
      if (pwmConfig.isNull())
        return false;
        
      bool configComplete = true;
      int8_t newPin = pin_;
      uint32_t newFreq = freq_;
      configComplete &= getJsonValue(pwmConfig[F("pin")], newPin);  
      configComplete &= getJsonValue(pwmConfig[F("freq")], newFreq);

      if (en)
        open(newPin, newFreq);
      else
        close();

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

class TimerServoUsermod;

struct cbInfo {
  TimerServoUsermod* content;
  esp_timer_handle_t* oneShotTimer;
};

cbInfo cbInfoData[2];

void cbFeedTime(void* arg);


class TimerServoUsermod : public Usermod {
  public:

    static const char USERMOD_NAME[];
    static const char _enable[];   // 是否开启
    static const char _feedTime1[]; // 喂食时间
    static const char _feedTime2[]; // 喂食时间
    static const char _dutyStart[];    // 舵机起始占空比
    static const char _dutyStep[];     // 占空比步进值
    static const char _dutyStepCurr[]; // 第几步

    bool enable = false;
    int feedTimes[2] {32400,  -1}; //
    float dutyAtStart = 0.05;
    float dutyStep = 0.0222222; // perAngle = 360/18=20; mapf(20, 0, 180, 0.05, 0.25)
    int dutyStepCurr = 0;

    esp_timer_handle_t oneshot_timer[2];
 

    void feed() {
      DEBUG_PRINTF("servo moving... feed now.\n");
      
      float nextDuty = dutyAtStart+dutyStep*dutyStepCurr;
      servoPwm.setDuty(nextDuty);

      dutyStepCurr++;
      dutyStepCurr = dutyStepCurr % 6;
    }

    uint64_t calcTime(int index) {
      if (feedTimes[index]<0)
        return 0;

      byte h = hour(localTime);
      byte m = minute(localTime);
      byte s = second(localTime);

      int64_t todayCurTotalSec = h*SECS_PER_HOUR + m*SECS_PER_MIN + s;
      int64_t targetSecAtDay = feedTimes[index]*SECS_PER_HOUR;

      int secs = 0;
      if (targetSecAtDay > todayCurTotalSec) {
        secs = targetSecAtDay-todayCurTotalSec; // 当天未到执行时间
      } else {
        secs = (SECS_PER_DAY - todayCurTotalSec) + targetSecAtDay; // 已过执行时间，第二天执行时间
      }

      return secs*1000; 
    }

    void updateTimer() {
      for (int i = 0; i < 2; ++i) 
      {
        if (esp_timer_is_active(oneshot_timer[i]))
          esp_timer_stop(oneshot_timer[i]);

        if (enable) {
          uint64_t ntime = calcTime(feedTimes[i]);
          if (ntime > 0)
            DEBUG_PRINTF("update timer. next feed time: %d s\n", (ntime/1000));

            ESP_ERROR_CHECK(esp_timer_start_once(oneshot_timer[i], ntime));
        }
      }
    }

    void setup() {
      for (int i = 0; i < 2; ++i) {
        cbInfoData[i].content = this;
        cbInfoData[i].oneShotTimer = &oneshot_timer[i];
        char name[128];
        sprintf(name, "one-shot-%d", i);
        // By default all PWM outputs are disabled, no setup do be done
        const esp_timer_create_args_t oneshot_timer_args = {
                .callback = cbFeedTime,
                /* argument specified here will be passed to timer callback function */
                .arg = (void*)&cbInfoData[i],
                .name = name
        };
        ESP_ERROR_CHECK(esp_timer_create(&oneshot_timer_args, &oneshot_timer[i]));
      }
    }

    void loop() {

    }

    void addToJsonState(JsonObject& root) {
    }

    void readFromJsonState(JsonObject& root) {
    }

    void addToJsonInfo(JsonObject& root) {
    }

    void addToConfig(JsonObject& root) {
      JsonObject top = root.createNestedObject(USERMOD_NAME);
      servoPwm.addToConfig(top);
      top[_enable      ] = enable;
      top[_feedTime1   ] = feedTimes[0];
      top[_feedTime2   ] = feedTimes[1];
      top[_dutyStart   ] = dutyAtStart;
      top[_dutyStep    ] = dutyStep;
      top[_dutyStepCurr ] = dutyStepCurr;
    }

    bool readFromConfig(JsonObject& root) {
      JsonObject top = root[USERMOD_NAME];
      if (top.isNull())
        return false;

      bool configComplete = true;
      configComplete &= getJsonValue(top[_enable], enable);  
      configComplete &= servoPwm.readFromConfig(top, enable); // 将打开 ledc 通道
      configComplete &= getJsonValue(top[_feedTime1   ], feedTimes[0]);  
      configComplete &= getJsonValue(top[_feedTime2   ], feedTimes[1]);  
      configComplete &= getJsonValue(top[_dutyStart   ], dutyAtStart);  
      configComplete &= getJsonValue(top[_dutyStep    ], dutyStep);  
      configComplete &= getJsonValue(top[_dutyStepCurr], dutyStepCurr);  

      if (enable) {
        float nextDuty = dutyAtStart+dutyStep*dutyStepCurr;
        servoPwm.setDuty(nextDuty);
      }

      dutyStepCurr++;
      dutyStepCurr = dutyStepCurr % 6;

      updateTimer(); // 启动定时器

      return configComplete;
    }

    uint16_t getId() {
      return USERMOD_ID_TIMER_SERVO;
    }

  private:
    PwmOutput servoPwm;
};

void cbFeedTime(void* arg)
{ 
  cbInfo* info = (cbInfo*)arg;

  DEBUG_PRINTF("timer trigger. start feed now\n");

  info->content->feed();

  DEBUG_PRINTF("set tomorrow feed.\n");

  ESP_ERROR_CHECK(esp_timer_start_once(*(info->oneShotTimer), SECS_PER_DAY*1000));
};

const char TimerServoUsermod::USERMOD_NAME[] PROGMEM = "TimerServoFeed";
const char TimerServoUsermod::_enable       [] PROGMEM = "Enable"; // 是否开启
const char TimerServoUsermod::_feedTime1    [] PROGMEM = "FeedTime1s"; // 喂食时间
const char TimerServoUsermod::_feedTime2    [] PROGMEM = "FeedTime2s"; // 喂食时间
const char TimerServoUsermod::_dutyStart    [] PROGMEM = "DutyAtStart"; // 舵机起始占空比
const char TimerServoUsermod::_dutyStep     [] PROGMEM = "DutyStep";    // 占空比步进值
const char TimerServoUsermod::_dutyStepCurr [] PROGMEM = "DutyStepCurr";    // 第几步



