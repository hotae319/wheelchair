#ifndef Wheelchair_H_
#define Wheelchair_H_

#include <Arduino.h>
#include "pinmap.h"
#include "constants.h"
#include "util.h"
#include "HX711.h"	// loadcell lib
#include "vescuino_uart_arduino.h"
#include "vescuino_common.h"
#include "matfunc.h"

float smoothing(float data, float tau, float ts); // peak removal
float saturation(float data, uint32_t limit); // block under limit
typedef struct cart_pose_
{
  float inclination; //theta
  float rotation; //phi
}cart_pose;

typedef struct imu_pose
{
  float _roll;
  float _pitch;
  float _yaw;
  float _theta;
  float _phi;
  float _phi_ref;
}imu_pose;

class Wheelchair {
  public:
    Wheelchair(VESC_UART* vesc_1, VESC_UART* vesc_2, VESC_CONTROL* vesc_ctrl) {
      this->_vesc_1 = vesc_1;
      this->_vesc_2 = vesc_2;
      this->_vesc_ctrl = vesc_ctrl;

      memset(loadcell_offset, 0, sizeof(loadcell_offset));
      memset(loadcell_value_raw, 0, sizeof(loadcell_value_raw));
      memset(loadcell_value_raw_prev, 0, sizeof(loadcell_value_raw_prev));
      memset(loadcell_value_lpf, 0, sizeof(loadcell_value_lpf));
      memset(loadcell_current, 0, sizeof(loadcell_current));
      memset(loadcell_current_pre, 0, sizeof(loadcell_current_pre)); // add

      //loadcell[0] = HX711(I_PIN_LOADCELL_LF_DAT, I_PIN_LOADCELL_LF_SCK);
      //loadcell[1] = HX711(I_PIN_LOADCELL_RF_DAT, I_PIN_LOADCELL_RF_SCK);
      loadcell[2] = HX711(I_PIN_LOADCELL_LB_DAT, I_PIN_LOADCELL_LB_SCK);
      loadcell[3] = HX711(I_PIN_LOADCELL_RB_DAT, I_PIN_LOADCELL_RB_SCK);

      state = BRAKE_SLEEP;
      next_state = BRAKE_SLEEP;
      side_state = NEUTRAL;
      side_timer = 0;

      memset(_enc_pos_stop, 0, sizeof(_enc_pos_stop));
      memset(_current, 0, sizeof(_current));
      memset(_rpm, 0, sizeof(_rpm));
      memset(_enc_pos, 0, sizeof(_enc_pos));
      memset(&_cart_pose, 0, sizeof(cart_pose));
      memset(&_imu_pose, 0, sizeof(imu_pose));

    }

    void setup();
    void run();

  private:
    cart_state_e state;
    cart_state_e next_state;

    side_state_e side_state;

    uint32_t side_timer;
    uint32_t brake_timer;

    VESC_UART* _vesc_1;
    VESC_UART* _vesc_2;
    VESC_CONTROL* _vesc_ctrl;

    //int32_t _tacho[2];
    //int32_t _tacho_stop[2];
    float _current[2];
    float *DisturbanceWheel;
    float _rpm[2];
    float _accel[2]; // estimation of mass and feedback gain
    float angle_acc[2];
    int32_t _enc_pos[2];
    int32_t _enc_pos_stop[2];
    float _desire_phi;
    cart_pose _cart_pose;
    imu_pose _imu_pose; 
    float *DesiredAcc;
    float DesiredSpd[2];
    float DesiredPos[2];
    



    int cal_idx = 0;
    int flag_cal = LOW;

    float current_gain;
    const float PID_LIMIT = 30;
    const float Kp = 0.1;//0.04; //1.2  0.25 // made 1/10
    const float Ki = 0.00;
    const float Kd = 0.004; //0.0004  ; //0.9  0.8 // made 1/10

    float err_prev[2] = {0,};
    float err_i[2] = {0,};

    void _pid_control();
    void _pid_reset();
    void g_compensation_control(float phi_ref, float theta, float *left, float *right, float d=D_MASSCENTER, float m=M_BODY); // compensate the gravity
    float* CompensationControl(float _rpm[], float phi_ref, float theta, float *DisturbanceTemp, float *desired_acc);

    float* DynamicObserver(float phi_ref, float theta, float _rpm[]);
    float* DisturbanceObserver(float phi_ref, float theta, float _rpm[], int32_t _enc_pos[], float _angle_acc[]);
    float* ParameterEstimator(float _angle_acc[], float _rpm[], float loadcell_current[]);
    float* DesiredMotion(float loadcell_current[], float _rpm[]);

    void calibration_reset();
    void calibration_charge();

    HX711 loadcell[LOADCELL_COUNT];

    int16_t loadcell_offset[LOADCELL_COUNT];
    int16_t loadcell_value_raw[LOADCELL_COUNT];
    int16_t loadcell_value_raw_prev[LOADCELL_COUNT];
    float   loadcell_value_lpf[LOADCELL_COUNT];
    float hand_force[LOADCELL_COUNT];

    int16_t loadcell_value_raw_calibration[LOADCELL_COUNT][50];
    
    float loadcell_current[2];
    float loadcell_current_pre[2]; // current previous value
    //float control_current[2]; // compensation or control current


    led_state_e loadcell_initialize();
    inline int16_t loadcell_read(int idx) {
      return (int16_t)(loadcell[idx].read() / 500);
    }
    inline int16_t loadcell_read_off(int idx) {
      return loadcell_read(idx) - loadcell_offset[idx];
    }

    int16_t loadcell_read_avg(int idx) {
      int32_t sum = 0;
      for (int j = 0; j < 16; j++) sum += loadcell_read(idx);
      return (int16_t)(sum / 16);
    }
    void loadcell_update();

    void auto_calibration();

    int is_touched();
    int LeftTouch();      //JW
    int MiddleTouch();      //JW
    int RightTouch();      //JW

    void brake_reset();
    void brake_position();
    void brake_rpm();

    int power_state;
    int button_state;
    int motor_state;
    float battery_volt;

    /* port */

    void _update_states();

    inline int _get_power_state() {
      this->power_state = digitalRead(I_PIN_POWER);
      return this->power_state;
    }

    inline float _get_battery_v() {
      this->battery_volt = util_lpf_battery(analogRead(I_PIN_BATTERY_ADC) * GAIN_BATTERY);
      return this->battery_volt;
    }

    inline int _get_button_state() {
      this->button_state = digitalRead(I_PIN_BUTTON);
      return (this->button_state);
    }
    void _get_imu_state();


    inline void _vesc_on() 	{
      digitalWrite(O_PIN_MOTOR_SWITCH, HIGH);
    }

    inline void _vesc_off() {
      digitalWrite(O_PIN_MOTOR_SWITCH, LOW);
      Serial.println("vesc off");
    }

    inline void _system_reset() {
      RSTC->RSTC_CR = 0xA5000005;
    }

    inline void _led1_on() 		{
      digitalWrite(O_PIN_LED1, HIGH);
    }
    inline void _led2_on() 		{
      digitalWrite(O_PIN_LED2, HIGH);
    }
    inline void _led3_on() 		{
      digitalWrite(O_PIN_LED3, HIGH);
    }
    inline void _led4_on() 		{
      digitalWrite(O_PIN_LED4, HIGH);
    }
    inline void _alert_led_on() {
      digitalWrite(O_PIN_ALERT_LED, HIGH);
    }

    inline void _led1_off() 	 {
      digitalWrite(O_PIN_LED1, LOW);
    }
    inline void _led2_off() 	 {
      digitalWrite(O_PIN_LED2, LOW);
    }
    inline void _led3_off() 	 {
      digitalWrite(O_PIN_LED3, LOW);
    }
    inline void _led4_off() 	 {
      digitalWrite(O_PIN_LED4, LOW);
    }
    inline void _alert_led_off() {
      digitalWrite(O_PIN_ALERT_LED, LOW);
    }

    inline void _all_led_on() {
      _led1_on();
      _led2_on();
      _led3_on();
      _led4_on();
    }

    inline void _all_led_off() {
      _led1_off();
      _led2_off();
      _led3_off();
      _led4_off();
    }

    void set_LED(led_state_e state);
};


#endif /* Wheelchair_H_ */






































































































































































































































































































































































































































