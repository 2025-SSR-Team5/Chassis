/// @file
#ifndef CHASSIS_MOTOR_H_
#define CHASSIS_MOTOR_H_

#ifdef __MBED__

#include <Pwm.h>
#include <mbed.h>

namespace rct {

/// @brief モータへ出力する。
/// @note コピームーブ不可
struct Motor : mbed::NonCopyable<Motor> {
  /// @brief コンストラクタ
  /// @param pinA 正転時のPWM出力ピン
  /// @param pinB 逆転時のPWM出力ピン
  Motor(const PinName pinA, const PinName pinB) : outA_{pinA}, outB_{pinB} {
    outA_.period_us(100);
    outB_.period_us(100);
  }

  /// @brief モータ出力をセットする。
  /// @param power モータ出力
  /// @pre -1.0 <= power <= 1.0
  void set_pwm(const float power) {
    pwm_ = power;
  }
  /// @brief PWMを出力する。
  void write() {
    outA_ = pwm_[0];
    outB_ = pwm_[1];
  }
  /// @brief シリアルモニタにモータ出力を表示する。
  void print() {
    printf("%d\t%d\t", int(pwm_[0] * 100), int(pwm_[1] * 100));
  }
  /// @brief モータ出力をセットしPWM出力する。
  /// @param val モータ出力
  /// @pre -1.0 <= val <= 1.0
  /// @return val
  float operator=(const float val) {
    set_pwm(val);
    write();
    return val;
  }
 private:
  mbed::PwmOut outA_;
  mbed::PwmOut outB_;
  Pwm pwm_ = {};
};

}  // namespace rct

#elif defined(ARDUINO)  // __MBED__

namespace rct {

/// @brief モータへ出力する。
struct Motor {
  /// @brief コンストラクタ
  /// @param pinA 正転時のPWM出力ピン
  /// @param pinB 逆転時のPWM出力ピン
  Motor(const int pinA, const int pinB) : pin_{pinA, pinB} {
    pinMode(pinA, OUTPUT);
    pinMode(pinB, OUTPUT);
  }
  Motor(const Motor&) = delete;
  Motor(Motor&&) = delete;
  /// @brief モータ出力をセットする。
  /// @param power モータ出力
  void set_pwm(const int power) {
    pwm_[0] = power > 0 ? power : 0;
    pwm_[1] = power > 0 ? 0 : -power;
  }
  /// @brief PWMを出力する。
  void write() {
    analogWrite(pin_[0], pwm_[0]);
    analogWrite(pin_[1], pwm_[1]);
  }
  /// @brief シリアルモニタにモータ出力を表示する。
  void print() {
    Serial.print(pwm_[0]);
    Serial.print('\t');
    Serial.print(pwm_[1]);
    Serial.print('\t');
  }
  /// @brief モータ出力をセットしPWM出力する。
  /// @param val モータ出力
  /// @return val
  int operator=(const int val) {
    set_pwm(val);
    write();
    return val;
  }
 private:
  int pin_[2];
  uint8_t pwm_[2] = {};
};

}  // namespace rct

// ESPは後から追加、ロジックが違うので注意
#elif defined(ESP_PLATFORM)

#include "driver/ledc.h"
#include "driver/gpio.h"

#define PWM_MODE LEDC_LOW_SPEED_MODE 
#define PWM_DUTY_RES LEDC_TIMER_10_BIT // Set duty resolution to 13 bits
#define PWM_FREQUENCY          (20000) // Frequency in Hertz. Set frequency at 20 kHz

namespace rct {

/// @brief モータへ出力する。
struct Motor {
  /// @brief コンストラクタ
  /// @param pinA PWM出力ピン
  /// @param pinB DIR出力ピン

  Motor(const int pinA, const int pinB) : pin_{pinA, pinB}, num(next_num++) {
    // ESP32-S3 has maximum 8 LEDC channels (0-7)
    if (num >= 8) {
        printf("Error: Too many Motor instances! Maximum 8 channels available.\n");
        return;
    }
    
    gpio_config_t io_conf = {};  // (1)
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = 1ULL << pinB;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_config(&io_conf);

    // Prepare and then apply the LEDC PWM timer configuration
    ledc_timer_config_t pwm_timer = {};
    pwm_timer.speed_mode = PWM_MODE;
    pwm_timer.duty_resolution = PWM_DUTY_RES;
    pwm_timer.timer_num = (ledc_timer_t)num;
    pwm_timer.freq_hz = PWM_FREQUENCY;
    pwm_timer.clk_cfg = LEDC_AUTO_CLK;
    ESP_ERROR_CHECK(ledc_timer_config(&pwm_timer));

    // Prepare and then apply the LEDC PWM channel configuration
    ledc_channel_config_t pwm_channel = {};
    pwm_channel.speed_mode = PWM_MODE;
    pwm_channel.channel = (ledc_channel_t)num;
    pwm_channel.timer_sel = (ledc_timer_t)num;
    pwm_channel.intr_type = LEDC_INTR_DISABLE;
    pwm_channel.gpio_num = pinA;
    pwm_channel.duty = 0;
    pwm_channel.hpoint = 0;
    ESP_ERROR_CHECK(ledc_channel_config(&pwm_channel));
  }
  Motor(const Motor&) = delete;
  Motor(Motor&&) = delete;
  /// @brief モータ出力をセットする。
  /// @param power モータ出力
  /// @brief モータ出力をセットする。
  /// @param power モータ出力
  void set_pwm(const float power) {
    pwm_ = power;
  }
  /// @brief PWMを出力する。
  void write() {
    if (pwm_ >= 0) {
        dir_ = 0;
    } else {
        dir_ = 1;
        pwm_ = -pwm_;
    }
    gpio_set_level((gpio_num_t)pin_[1], dir_);
    // Set duty to pwm_ * max_duty (10-bit resolution = 1024)
    ESP_ERROR_CHECK(ledc_set_duty(PWM_MODE, (ledc_channel_t)num, (1 << 10)*pwm_));
    // Update duty to apply the new value
    ESP_ERROR_CHECK(ledc_update_duty(PWM_MODE, (ledc_channel_t)num));
  }
  /// @brief シリアルモニタにモータ出力を表示する。
  void print() {
    printf("%d\t%d\t", int(pwm_ * 100), dir_);
  }
  /// @brief モータ出力をセットしPWM出力する。
  /// @param val モータ出力
  /// @pre -1.0 <= val <= 1.0
  /// @return val
  float operator=(const float val) {
    set_pwm(val);
    write();
    return val;
  }
  private:
    int pin_[2];
    float pwm_ = 0;
    int dir_ = 0;

    int num;
    static int next_num;
};

int Motor::next_num = 0;

}


#endif //defined()

#endif  // CHASSIS_MOTOR_H_
