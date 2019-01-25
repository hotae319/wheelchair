#include "Wheelchair.h"
#include "flash.h"
#include "MPU9250_wheelchair.h"

#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
  #include <avr/power.h>
#endif
#define PIN            5

int _touch_count = 0;
uint32_t stop_t;
bool straight_flag;
float lpf_straight(float data, float tau, float ts){
  static float pre_value = 0; float lpf_value1;
  if(straight_flag){
    pre_value = 0;
  }
  lpf_value1 = (tau * pre_value + ts * data) / (tau + ts);
  pre_value = lpf_value1;
  return lpf_value1;
}
float* lpf_dob(float data[], float tau, float ts){
  static float pre_value[2] = {0,}; float lpf_value;
  for(int i = 0; i < 2; i++){
    lpf_value = (tau * pre_value[i] + ts * data[i]) / (tau + ts);
    pre_value[i] = lpf_value;
  }
  return pre_value;
}
float saturation(float data, uint32_t limit){
  data = limit;
  return data;
} 
float sgn(float a){
  if(a>0){
    return 1;
  }
  else if(a<0){
    return -1;
  }
  else{
    return 0;
  }
}


Adafruit_NeoPixel pixels = Adafruit_NeoPixel(30, PIN, NEO_GRB + NEO_KHZ800);

void Wheelchair::setup() {

  pixels.begin();	
  //Wire1.begin(); 
  pinMode(I_PIN_POWER, INPUT);
  pinMode(I_PIN_BUTTON, INPUT);
  pinMode(O_PIN_LED1, OUTPUT);
  pinMode(O_PIN_LED2, OUTPUT);
  pinMode(O_PIN_LED3, OUTPUT);
  pinMode(O_PIN_LED4, OUTPUT);
  pinMode(O_PIN_ALERT_LED, OUTPUT);
  pinMode(O_PIN_MOTOR_SWITCH, OUTPUT);

  _all_led_off();
  _alert_led_off();
  _vesc_on();
  analogRead(I_PIN_BATTERY_ADC) * GAIN_BATTERY;
  delay(50);
  flag_cal = LOW;
  // battery voltage Low pass filter
  util_lpf_battery_set_prevalue(analogRead(I_PIN_BATTERY_ADC) * GAIN_BATTERY);

  led_state_e ret;
  if ((ret = loadcell_initialize()) != START_UP) {
    // when reading loadcell, START_UP, FACTORY, ERROR_CRC 
    while (true) {
      set_LED(ret); 
      
      if (_get_button_state() == BUTTON_STATE_PRESS) {
        calibration_reset();
      }
      
    }
  }
  // motor switch high
  _vesc_on();
  _vesc_1->init();
  _vesc_2->init();
  util_delay_sec(3); 

  set_LED(START_UP);
  _all_led_off();

  mpu9250_wheelchair_setup();
  this->_cart_pose.rotation = 0; //initialize
  this->_enc_pos[0] = 0;
  this->_enc_pos[1] = 0;
  // loadcell seting
  /*
  for(int i = 0; i<4; i++){
      loadcell[i].set_scale();
      loadcell[i].tare();
  }*/

}

void Wheelchair::run() {
  ;
  static int straight_count = 0;
  static int emer = 0; // emergency is divided into obstacle(0)/bump(1)
  int i;
  if(print_flag){    
    
    Serial.print("currentR, currentL, vesc_currentR, vesc_currentL, encR, encL, rotation, inclination: ");
    Serial.print(loadcell_current[1]); // right (front +)
    Serial.print(","); 
    Serial.print(loadcell_current[0]); // left (front -)
    Serial.print(",");
    Serial.print(_current[1]);
    Serial.print(",");
    Serial.print(_current[0]);
    Serial.print(",");
    Serial.print(this->_enc_pos[1]);
    Serial.print(",");
    Serial.print(this->_enc_pos[0]);
    Serial.print(",");
    Serial.print(this->_cart_pose.rotation);
    Serial.print(",");
    Serial.println(_cart_pose.inclination);
    Serial.print("state, right_accel, voltage, is_touched:");
    Serial.print(state);
    Serial.print(",");
    Serial.print(_accel[1]);
    Serial.print(",");
    Serial.print(this->battery_volt);
    Serial.print(",");
    Serial.println(is_touched());
    Serial.print("_imu_pose._phi, _imu_pose._phi_ref");    
    Serial.print(_imu_pose._phi); // left
    Serial.print(",");
    Serial.println(_imu_pose._phi_ref);
    Serial.print("rpm:");
    Serial.println(_rpm[1]);
    Serial.print("hand_force_R, hand_force_L");
    Serial.print(loadcell_current[1]*500/LOADCELL_CURRENT_GAIN_B*0.4535/7050*0.182*9.81); //0.957*loadcell_current
    Serial.print(",");
    Serial.print(-loadcell_current[0]*500/LOADCELL_CURRENT_GAIN_B*0.4535/7050*0.182*9.81);
    Serial.println("N");
    Serial.print("voltage");
    Serial.println(analogRead(A8));
    
    print_flag = false;

  }
  // state check
  _update_states();
  
  // state action
  if (state != next_state) {
    state = next_state;
    // transition action
    switch (state) {
      case RUN:
        _vesc_on();
        brake_timer = millis();
        break;
        straight_count = 0;
      case BRAKE_1:
        _vesc_on();
        brake_timer = millis();
        break;

      case BRAKE_2:
        _vesc_on();
        _pid_reset();
        stop_t = millis();
        brake_timer = millis();
        _enc_pos_stop[0] =  _vesc_ctrl->enc_pos[1];
        _enc_pos_stop[1] =  _vesc_ctrl->enc_pos[2];
        break;

      case BRAKE_3:
        _vesc_on();

        _enc_pos_stop[0] =  _vesc_ctrl->enc_pos[1];
        _enc_pos_stop[1] =  _vesc_ctrl->enc_pos[2];
        break;
      case BRAKE_SLEEP:
        _vesc_1->set_current(0);
        _vesc_2->set_current(0);
        break;
      case EMERGENCY:
        _vesc_on();
        brake_timer = millis();
        break;
      case STRAIGHT:
        _vesc_on();
        brake_timer = millis();
        _desire_phi = _cart_pose.rotation;
        straight_flag = true;
        break;
    }
  } else {
    // loop action
    switch (state) {
      case RUN:
      {
        straight_flag = false;
        //auto_calibration();
        set_LED(VBAT);
        float left_current=0, right_current=0;
        if(_cart_pose.inclination>10){
          //compensation_control(_imu_pose._phi_ref/180.0f*PI, _imu_pose._theta/180.0f*PI, &left_current, &right_current);
        }
        if(abs(loadcell_current[0]+loadcell_current[1])<ST_INTENTION){
          straight_count ++;
        }
        else{
          straight_count -= 2;
        }
        
        DisturbanceWheel = DynamicObserver(_imu_pose._phi_ref, _imu_pose._theta, _rpm);
        _vesc_1->set_current(loadcell_current[1]+right_current);
        _vesc_2->set_current(loadcell_current[0]-left_current);

        if(print_flag){
          Serial.print("DisturbanceWheel R,L: ");
          Serial.print(DisturbanceWheel[1]);
          Serial.print(",");
          Serial.println(DisturbanceWheel[0]);
          
          /*
          Serial.print("left_current, right_current:");
          Serial.print(left_current);
          Serial.print(",");
          Serial.println(right_current);*/
        }
        //delete[] DisturbanceWheel;
        //DisturbanceWheel = nullptr;
        break;
      }   

      case BRAKE_1:
        set_LED(VBAT);
        _vesc_1->set_current_brake(25);
        _vesc_2->set_current_brake(25);
        break;

      case BRAKE_2:
        set_LED(VBAT);
        brake_position();        
        if (abs(_current[0]) >= 3 || abs(_current[1]) >= 3) {
          brake_timer = millis();  // if force is still applied, brake_timer extends
        }
        break;

      case BRAKE_3:
        set_LED(VBAT);
        _vesc_1->set_pos(_enc_pos_stop[1]);
        _vesc_2->set_pos(_enc_pos_stop[0]);
        //brake_position();
        //brake_rpm();        
        if (abs(_current[0]) >= 3 || abs(_current[1]) >= 3) {
          brake_timer = millis();  // if force is still applied, brake_timer extends
        }
        break;

      case BRAKE_SLEEP:
        set_LED(VBAT);

        break;

      case EMERGENCY:
        if(emer==0){
          // remove the additional torque
          _vesc_1->set_current(0);
          _vesc_2->set_current(0);
          // offer feedback to user
          //_vesc_1->set_current(DNAGER_FEEDBACK*loadcell_current[1]); // reverse
          //_vesc_2->set_current(-DNAGER_FEEDBACK*loadcell_current[0]); // reverse
        }          
        else if(emer==1){          
          // lpf_straight the skyrocketed value
          float smooth_current1 = lpf_straight(loadcell_current[1], 0.1, 0.015); // have to make lpf_straight ftn.
          float smooth_current2 = lpf_straight(loadcell_current[0], 0.1, 0.015);
          _vesc_1->set_current(smooth_current1);
          _vesc_2->set_current(smooth_current2);          
        }      
        // soon after we can add impedance control in case of emergency
        break;
      case STRAIGHT:
        {
          float left_correction =0;
          float right_correction = 0;
          //straight_correction(_desire_phi, &left_correction, &right_correction);
          straight_flag = false;
          if(abs(loadcell_current[0]+loadcell_current[1])<ST_INTENTION){
            straight_count ++;
          }
          else{
            straight_count =0;
          }
          
          _vesc_1->set_current(loadcell_current[1]+right_correction);
          _vesc_2->set_current(loadcell_current[0]+left_correction);
          /*//if(print_flag){
            Serial.print("straight /// left_correction, right_correction:");
            Serial.print(left_correction);
            Serial.print(",");
            Serial.println(right_correction);
          //*/}
          break;
        }

    }
    if (power_state == POWER_STATE_CHARGING) calibration_charge();
    if (button_state == BUTTON_STATE_PRESS) calibration_reset();

    uint32_t time_out = millis() - brake_timer;
    if(print_flag){
      Serial.print("timeout");
      Serial.println(time_out);      
    }

    // state transition
    switch (state) {
      case STRAIGHT:
        if (is_touched() && straight_count<ST_COUNT){
          brake_timer = millis();
          next_state = RUN;
          
          break;
        }
        else if(is_touched() && straight_count>=ST_COUNT){
          brake_timer = millis();
          break;
        }
        else if (!is_touched() && time_out > 300){
          next_state = BRAKE_1;
          break;
        }
      case EMERGENCY:
        if (abs(loadcell_current[0])-abs(loadcell_current_pre[0])<20){
          // finish the emergency          
          next_state = RUN;   
          break;     
        }
      case RUN:
        if (is_touched() && straight_count < ST_COUNT) {
          brake_timer = millis();
          break;
          // possible to change the condition due to distinguishing start with this
          // or start climbing
          /*
          if (abs(loadcell_current[0])>OBSTACLE_CURRENT_LIMIT && abs(loadcell_current[1])>OBSTACLE_CURRENT_LIMIT && abs(loadcell_current[1]-loadcell_current_pre[1])>OBSTACLE_CURRENT_GAP && abs(loadcell_current[0]-loadcell_current_pre[0])>OBSTACLE_CURRENT_GAP){
          // obstacle in front of wheelchair
          emer = 0;
          next_state = EMERGENCY;
          break;
          }
          else if ((abs(loadcell_current[1]-loadcell_current_pre[1])>BUMP_CURRENT_LIMIT) && abs(loadcell_current[0]-loadcell_current_pre[0])>BUMP_CURRENT_LIMIT){
            // bump
          emer = 1;
          next_state = EMERGENCY;
          break;
          }*/
        } 
        else if(is_touched() && straight_count>=ST_COUNT){
          brake_timer = millis();
          next_state = STRAIGHT;          
          break;
        }
        else if (time_out > 300) {
          next_state = BRAKE_1;
        }                 
      case BRAKE_1:
        if (is_touched()) {
          next_state = RUN;
        } else if (time_out > 500) {
          next_state = BRAKE_2;
        }
      case BRAKE_2:
        if (is_touched()) {
          next_state = RUN;
        } else if (time_out > 2000) {
          next_state = BRAKE_3;
        }
      case BRAKE_3:
        if (is_touched()) {
          next_state = RUN;
        } else if (time_out > 6000) {
          next_state = BRAKE_SLEEP;
        }
      case BRAKE_SLEEP:
        if (is_touched()) {
          next_state = RUN;          
        }

    }

  }



int Wheelchair::is_touched() {
  if (abs(loadcell_current[0]) >= LOADCELL_CURRENT_DEADZONE ||
      abs(loadcell_current[1]) >= LOADCELL_CURRENT_DEADZONE) {
      _touch_count++;
  }
  else {
    _touch_count = 0;
  }

  if (_touch_count >= 5) {
    return 1;
  }

  else {
    return 0;
  }

}

void Wheelchair::_update_states() {    
  float pre_enc_pos[2]; 
  float pre_rpm[2];  
  uint32_t t_now; 
  float odometry; float correction;
  static int j = 0;
  static uint32_t t_pre = 0; 
  static float _imu_pose_phi_pre;
  static float xy=0; 
  static float xsquare=0;

  // two states(digital) battery(analog)
  _get_power_state();
  _get_battery_v();
  _get_button_state();  

  _vesc_1->get_values();
  _vesc_2->get_values();

  t_now = millis();  
  for (int i = 0; i < 2; i++) {
    this->_current[i] 	= _vesc_ctrl->current[i+1];
    pre_rpm[i] = this->_rpm[i];
    pre_enc_pos[i] = this->_enc_pos[i];
    this->_rpm[i] = _vesc_ctrl->erpm[i+1]/POLE_PAIR;
    this->_enc_pos[i] = _vesc_ctrl->enc_pos[i+1];
    Serial.print("enc_pos: ");
    Serial.println(_enc_pos[i]);
    if( _rpm[i] != pre_rpm[i]){
      this->_accel[i] = float((this->_rpm[i] - pre_rpm[i])/(t_now-t_pre)*1000.0/60*2*PI); //rad/s^2
      /*
      Serial.print("accel, current, t_now, t_pre:");
      Serial.print(_accel[i]);
      Serial.print(",");
      Serial.print(loadcell_current[i]);
      Serial.print(",");
      Serial.print(t_now);
      Serial.print(",");
      Serial.println(t_pre);
      */
      if(i==1){
        t_pre = millis();
      }
      // enough with angle acceleration
    }
    else{
      this->_accel[i] = 0;
      //keep t_pre
    }
  }

  if (j==0){
    this->_enc_pos[0] = 0;
    this->_enc_pos[1] = 0; //initialization
    current_gain = 0;
    j++;
  }
  
  _get_imu_state(); // get imu data, store the data on the global variable
  // pose estimation update
  
  this->_cart_pose.inclination = _imu_pose._theta; // from imu data 
  //this->_cart_pose.rotation = float((this->_enc_pos[0]-this->_enc_pos[1])/DIST_WHEELS)*RADIUS_WHEEL; //
  float err1 = _enc_pos[1]-pre_enc_pos[1];
  float err2 = _enc_pos[0]-pre_enc_pos[0];  
  if((_enc_pos[1]-pre_enc_pos[1])*_rpm[1] < 0 && abs(_enc_pos[1]-pre_enc_pos[1])>200){
    err1 -= 360*sgn(_enc_pos[1]-pre_enc_pos[1]);
  }
  else if((_enc_pos[0]-pre_enc_pos[0])*_rpm[0] < 0 && abs(_enc_pos[0]-pre_enc_pos[0])>200){
    err2 -= 360*sgn(_enc_pos[0]-pre_enc_pos[0]);
  }
  odometry = (err1+err2)/DIST_WHEELS*RADIUS_WHEEL;
   
  // because  left and right wheel has different direction
  if(print_flag){
  Serial.print("odometry:");
  Serial.println(odometry); // right
  }
  correction = 0.98*(_imu_pose._phi-_imu_pose_phi_pre) + 0.02*odometry;  
  if(_cart_pose.inclination<10){
    this->_cart_pose.rotation =_imu_pose_phi_pre + correction;
    _imu_pose_phi_pre = _imu_pose._phi;
  }
  else{
    this->_cart_pose.rotation = _imu_pose_phi_pre - correction; // minus due to direction
    _imu_pose_phi_pre = _imu_pose._phi_ref; 
  }
    /*t_now = millis();
  // this->_cart_pose.rotation += ((__rpm[0]-pre__rpm[0])-(__rpm[1]-pre__rpm[1]))/DIST_WHEELS*RADIUS_WHEEL*(t_now-t_pre);
  t_pre = millis();*/
  //this->_cart_pose.rotation = this->_cart_pose.rotation - int(this->_cart_pose.rotation/360)*360;
  this->loadcell_update();
  
  if(is_touched() && j<50 && abs(_accel[1])>0.5){
  	// linear regression LSM
  	xy += _accel[1]*_current[1];
  	xsquare += _accel[1]*_accel[1];
    current_gain = xy/xsquare *(LOADCELL_CURRENT_GAIN_B+1)*4/6-1;
    //if(current_gain<LOADCELL_CURRENT_GAIN_B){ current_gain = LOADCELL_CURRENT_GAIN_B}
    j++;
    Serial.print("j:");
    Serial.print(j);
    Serial.print(",");
    Serial.print("current_gain:");
    Serial.println(current_gain);
  }
  
}

void Wheelchair::_get_imu_state() {
	static uint16_t imu_count = 0; 
  mpu9250_wheelchair_act(_accel[0],_accel[1]); 
  //_imu_pose._roll = get_roll();
  //_imu_pose._pitch = get_pitch();
  //_imu_pose._yaw = get_yaw();  
  if(imu_count>200){
 	_imu_pose._theta = get_theta();
  }
  else{
  	_imu_pose._theta = atan(sqrtf(ax*ax+ay*ay)/abs(az));
  	imu_count++;
  }
	_imu_pose._phi = get_phi();
	_imu_pose._phi_ref = get_phi_ref();  
}
void Wheelchair::loadcell_update() {
  for (int i = 2; i < LOADCELL_COUNT; i++)
  {
    int16_t temp;
    temp = loadcell_read_off(i); // subtract offset
    //loadcell[i].set_scale(calibration_factor);
    //hand_force[i] = loadcell[i].get_units()*9.81; 
    // kg to N  2(left, minus =front), 3(right, plus = front)
    //     Reading[i] = temp;

    if (abs(loadcell_value_raw_prev[i]) <= LOADCELL_RAW_ZERO_DEADZONE) { 
      if (abs(temp - loadcell_value_raw_prev[i]) < LOADCELL_RAW_ZERO_NOISE_THRES) {
        loadcell_value_raw[i] = temp;
        loadcell_value_raw_prev[i] = loadcell_value_raw[i];
      }
    }
    else if (abs(temp - loadcell_value_raw_prev[i]) < LOADCELL_RAW_NOISE_THRES) {
      loadcell_value_raw[i] = temp;
      loadcell_value_raw_prev[i] = loadcell_value_raw[i];
    }
    else {
      loadcell_value_raw[i] = loadcell_value_raw_prev[i]; // if too large diff., it is regarded as error noise
    }

    if (abs(loadcell_value_raw[i]) < LOADCELL_RAW_DEADZONE)
      loadcell_value_raw[i] = 0;
    else if (loadcell_value_raw[i] >= LOADCELL_RAW_DEADZONE)
      loadcell_value_raw[i] -= LOADCELL_RAW_DEADZONE;
    else if (loadcell_value_raw[i] <= -LOADCELL_RAW_DEADZONE)
      loadcell_value_raw[i] += LOADCELL_RAW_DEADZONE;

    loadcell_value_lpf[i] = util_lpf(loadcell_value_raw[i], i);
  }

  // side detect
  static int flag_front = LOW, flag_back = LOW;

  if (abs(loadcell_value_lpf[LF]) >= LOADCELL_RAW_DEADZONE ||
      abs(loadcell_value_lpf[RF]) >= LOADCELL_RAW_DEADZONE) {
    flag_front = HIGH;
  }
  else if (abs(loadcell_value_lpf[LF]) < LOADCELL_RAW_DEADZONE &&
           abs(loadcell_value_lpf[RF]) < LOADCELL_RAW_DEADZONE) {
    flag_front = LOW;

  }

  if (abs(loadcell_value_lpf[LB]) >= LOADCELL_RAW_DEADZONE ||
      abs(loadcell_value_lpf[RB]) >= LOADCELL_RAW_DEADZONE) {
    flag_back = HIGH;

  }
  else if (abs(loadcell_value_lpf[LB]) < LOADCELL_RAW_DEADZONE &&
           abs(loadcell_value_lpf[RB]) < LOADCELL_RAW_DEADZONE) {
    flag_back = LOW;
  }


  switch (side_state)
  {
    case NEUTRAL:

      side_timer = millis();
      if (flag_front == HIGH && flag_back == LOW )
        side_state = FORWARD;
      else if (flag_back == HIGH && flag_front == LOW)
        side_state = BACKWARD;
      break;

    case FORWARD:
      if (flag_front == LOW) {
        if ((millis() - side_timer) > 500)
          side_state = NEUTRAL;
      }
      else {
        side_timer = millis();
      }

      break;

    case BACKWARD:
      if (flag_back == LOW) {
        if ((millis() - side_timer) > 500)
          side_state = NEUTRAL;
      }
      else {
        side_timer = millis();
      }
      break;
  }

  // current transform

  switch (side_state) {
    case NEUTRAL:
    case FORWARD:
      for (int i = 0; i < 2; i++) {
        loadcell_current_pre[i] = loadcell_current[i]; //restore the previous value
        loadcell_current[i] = float(loadcell_value_lpf[i]) * LOADCELL_CURRENT_GAIN_F;
      }
      break;
    case BACKWARD:
      for (int i = 0; i < 2; i++) {
        loadcell_current_pre[i] = loadcell_current[i];
        loadcell_current[i] = float(loadcell_value_lpf[i + 2]) * LOADCELL_CURRENT_GAIN_B;        
      }
      break;
  }
}

void Wheelchair::calibration_reset() {
  // read 16tiems and average them
  //loadcell_offset[LF] = loadcell_read_avg(LF);
  //loadcell_offset[RF] = loadcell_read_avg(RF);
  loadcell_offset[LB] = loadcell_read_avg(LB);
  loadcell_offset[RB] = loadcell_read_avg(RB);

  flash_loadcell(loadcell_offset);

  set_LED(CALIBRATION);
  _system_reset();
}

void Wheelchair::calibration_charge() {
  const int WINDOW_LOCAL = 10;
  const int WINDOW_GLOBAL = 300;

  int16_t local_arr[LOADCELL_COUNT][WINDOW_LOCAL] = {0};
  int16_t global_arr[LOADCELL_COUNT][WINDOW_GLOBAL] = {0};

  int16_t calibrated_loadcell[4] = {0};

  int local_idx = 0;
  int global_idx = 0;

  uint32_t timer = millis();

  static int mode = 0;


  _all_led_off();
  _alert_led_off();
  _vesc_off();

  while (true) {

    if (_get_power_state() == POWER_STATE_BATTERY) {
      _system_reset();
    }
    else {
      set_LED(CHG);
      if (millis() - timer > 1000) {
        for (int i = 0; i < LOADCELL_COUNT; i++) {
          local_arr[i][local_idx] = loadcell_read(i);
        }
        local_idx = (local_idx + 1) % WINDOW_LOCAL;

        if (local_idx == 0) {
          int has_outlier = 0;

          for (int i = 0; i < 4; i++)
          {
            util_sort(local_arr[i], WINDOW_LOCAL);
            if (util_outlier_detect(local_arr[i], WINDOW_LOCAL))
            {
              for (int i = 0; i < 4; i++) memset(local_arr[i], 0, sizeof(local_arr[i]));
              has_outlier = 1;
              break;
            }
          }

          if (!has_outlier) {
            for (int i = 0; i < 4; i++) {
              global_arr[i][global_idx] = util_average_window(local_arr[i], 3, 4);
            }

            global_idx = (global_idx + 1) % WINDOW_GLOBAL;

            if (global_idx == 0) {
              int has_global_outlier = 0;

              for (int i = 0; i < 4; i++) {
                util_sort(global_arr[i], WINDOW_GLOBAL);
                if (util_outlier_detect(global_arr[i], WINDOW_GLOBAL))
                {
                  for (int i = 0; i < 4; i++) memset(global_arr[i], 0, sizeof(global_arr[i]));
                  has_global_outlier = 1;
                  break;
                }
              }

              if (!has_global_outlier) {
                for (int i = 0; i < 4; i++) {
                  calibrated_loadcell[i] = util_average_window(global_arr[i], 74, 150);
                }
                flash_loadcell(calibrated_loadcell);

                while (true) {
                  set_LED(CHG);
                  delay(50);
                }
              }
            }
          }
        }

        timer = millis();
      }
    }
  }
}

void Wheelchair::_pid_control() {
  float pid_out[2] = {0};
  static uint32_t pid_start_t = stop_t;
  #define ERR_DEAD 3
  for (int i = 0; i < 2; i++)
  {

    //int rot_count = int(_rpm[i]/60 * (millis()-pid_start_t)/1000);
    float err = _enc_pos[i] - _enc_pos_stop[i];
    if(abs(err)>240){ //err*sgn(_rpm[i])<0
      err -= 360*sgn(err);
    }
    /*
    if(abs(err) >300){
      err -= 360*sgn(err);
    } */   


    float err_p = err * Kp;
    float err_d = Kd * (err - err_prev[i]) / (millis()-pid_start_t) *1000; // delta_t = 0.015

    if(print_flag){
      /*
      Serial.print("err, err_d:");
      Serial.print(err);  
      Serial.print(",");    
      Serial.println(err_d);*/
    }

    err_i[i] += err * Ki * 0.015;
    pid_out[i] = err_p + err_i[i] + err_d;
    err_prev[i] = err;
    if(i==1){
	    pid_start_t = millis();
    }

    if (pid_out[i] > PID_LIMIT)         pid_out[i] = PID_LIMIT;
    else if (pid_out[i] < -PID_LIMIT)   pid_out[i] = -PID_LIMIT;
  }
  _vesc_1->set_current(-pid_out[1]);
  _vesc_2->set_current(-pid_out[0]);

}


void Wheelchair::_pid_reset() {
  memset(err_prev, 0, sizeof(err_prev));
  memset(err_i, 0, sizeof(err_i));
}

void Wheelchair::compensation_control(float phi_ref, float theta, float *left, float *right, float d, float m){
  *left = 1.0/2.0*(d/DIST_WHEELS*sin(phi_ref)-1.0/2.0*cos(phi_ref))*m*gravity*sin(theta)*RADIUS_WHEEL/K_t;
  *right = -1.0/2.0*(d/DIST_WHEELS*sin(phi_ref)+1.0/2.0*cos(phi_ref))*m*gravity*sin(theta)*RADIUS_WHEEL/K_t;
}

void Wheelchair::straight_correction(float desire_phi, float *left_correction, float *right_correction){
  float err_yaw, err_d_yaw;
  float kp_s = 1.2, kd_s = 0.1;  
  err_yaw = _cart_pose.rotation - desire_phi;
  err_d_yaw = gz;
  if(print_flag){
    Serial.print("err,err_d,desire_phi");
    Serial.print(err_yaw);
    Serial.print(",");
    Serial.print(err_d_yaw);
    Serial.print(",");
    Serial.println(desire_phi);
  }
  //if(sgn(err_d_yaw)*err_yaw<0 && abs(err_yaw)>250) 
  // in original sgn(err_d)*sgn(err)<0 but err_d fluctuate +/-
  if(abs(err_yaw)>200) 
  {
    err_yaw -= 360*sgn(err_yaw);
  }
  if(abs(err_yaw) > 0.5){
    *left_correction = lpf_straight(-(kp_s*err_yaw+kd_s*gz),0.1,0.015);
    *right_correction = *left_correction;
  }
  else{
    *left_correction = 0;
    *right_correction = 0;
  }
}

float (*Transpose32(float matrix[][3]))[2]{
  static float TranMat[3][2];
  for(int i = 0; i++; i<3){
    for(int j = 0; j++; j<2){
      TranMat[i][j] = matrix[j][i];
    }
  }
  return TranMat;
}

float (*Transpose33(float matrix[][3]))[3]{
  static float TranMat[3][3];
  for(int i = 0; i++; i<3){
    for(int j = 0; j++; j<3){
      TranMat[i][j] = matrix[j][i];
    }
  }
  return TranMat;
}

float (*InverseMatrix2(float matrix[][2]))[2]{
  static float inv[2][2];
  float det = matrix[0][0]*matrix[1][1] - matrix[0][1]*matrix[1][0];
  inv[0][0] = matrix[1][1]/det;
  inv[0][1] = -matrix[0][1]/det;
  inv[1][0] = -matrix[1][0]/det;
  inv[1][1] = matrix[0][0]/det;
  return inv;
}
float (*InverseMatrix3(float matrix[][3]))[3]{
  static float inv[3][3];
  float (*Tmatrix)[3];
  float det = matrix[0][0]*matrix[1][1]*matrix[2][2] + matrix[1][0]*matrix[2][1]*matrix[0][2] + matrix[2][0]*matrix[0][1]*matrix[1][2]
  -matrix[0][0]*matrix[2][1]*matrix[1][2] - matrix[2][0]*matrix[1][1]*matrix[0][2] - matrix[1][0]*matrix[0][1]*matrix[2][2];
  Tmatrix = Transpose33(matrix);
  // inverse formula
  for(int i = 0; i++; i<3){
    for(int j = 0; j++; j<3){      
      inv[i][j] = 1.0/det*pow(-1, i+j)*(Tmatrix[(i-1)%3][(j-1)%3]*Tmatrix[(i+1)%3][(j+1)%3] - Tmatrix[(i+1)%3][(j-1)%3]*Tmatrix[(i-1)%3][(j+1)%3]);
    }
  }
  return inv;
}

float* Wheelchair::DynamicObserver(float phi_ref, float theta, int32_t _rpm[]){
  // It is about small state which includes only angle acceleration 
  float K1 = 0.8; float TorqueInput[2];
  //float *DisturbanceTemp = new float[2]; // dynamic allocation
  float *DisturbanceTemp;
  float xk[2], uk[2], yk[2]; float Kalman_Gain[2][2] = {0,};
  float Qk[2][2]={
    {1, 0},
    {0, 1}
  };
  float Rk[2][2] = {
    {2, 0},
    {0, 2}
  };
  for(int i=0; i<2; i++){
    TorqueInput[i] = loadcell_current[i]*K_t;
  }
  // measurement from ax and encoder 
  angle_acc[1] = K1*_accel[1] + (1-K1)*(ax/RADIUS_WHEEL-_accel[0]);
  angle_acc[0] = K1*_accel[0] + (1-K1)*(ax/RADIUS_WHEEL-_accel[1]);  
  
  DisturbanceTemp = SlopeControlDob(phi_ref, theta, _rpm, _enc_pos, angle_acc); // first apply DOB
  //Serial.print("Dis: ");
  //Serial.println(DisturbanceTemp[0]); 
  // Kalman filtering state equation, allocate uk

  /*uk[1] =  ((M_BODY*DIST_WHEELS*DIST_WHEELS*RADIUS_WHEEL*RADIUS_WHEEL + 4*I_WHEEL*DIST_WHEELS*DIST_WHEELS + I_BODY*RADIUS_WHEEL*RADIUS_WHEEL)
    *(DisturbanceTemp[1] + TorqueInput[1] + (M_BODY*RADIUS_WHEEL*gravity*cos(phi_ref)*sin(theta))/2 - (D_MASSCENTER*RADIUS_WHEEL*RADIUS_WHEEL*gz*_rpm[0]*(M_BODY - 2*M_WHEEL))/(2*DIST_WHEELS)))
  /((2*I_WHEEL*DIST_WHEELS*DIST_WHEELS + I_BODY*RADIUS_WHEEL*RADIUS_WHEEL)*(M_BODY*RADIUS_WHEEL*RADIUS_WHEEL + 2*I_WHEEL))
   + (RADIUS_WHEEL*RADIUS_WHEEL*(- M_BODY*DIST_WHEELS*DIST_WHEELS + I_BODY)*((D_MASSCENTER*gz*_rpm[1]*(M_BODY - 2*M_WHEEL)*RADIUS_WHEEL*RADIUS_WHEEL)/(2*DIST_WHEELS) 
    + (M_BODY*gravity*cos(phi_ref)*sin(theta)*RADIUS_WHEEL)/2 + DisturbanceTemp[0] + TorqueInput[0]))/((2*I_WHEEL*DIST_WHEELS*DIST_WHEELS + I_BODY*RADIUS_WHEEL*RADIUS_WHEEL)*(M_BODY*RADIUS_WHEEL*RADIUS_WHEEL + 2*I_WHEEL));
  */
   uk[1] = ((M_BODY*DIST_WHEELS*DIST_WHEELS*RADIUS_WHEEL*RADIUS_WHEEL + 4*I_WHEEL*DIST_WHEELS*DIST_WHEELS + I_BODY*RADIUS_WHEEL*RADIUS_WHEEL)*(DisturbanceTemp[1] 
    + TorqueInput[1] + (M_BODY*RADIUS_WHEEL*gravity*cos(phi_ref)*sin(theta))/2 - (D_MASSCENTER*RADIUS_WHEEL*RADIUS_WHEEL*gz*_rpm[0]/60.0*2*PI*(M_BODY - 2*M_WHEEL))
    /(2*DIST_WHEELS) - (D_MASSCENTER*M_BODY*RADIUS_WHEEL*gravity*sin(phi_ref)*sin(theta))/(2*DIST_WHEELS)))/((2*I_WHEEL*DIST_WHEELS*DIST_WHEELS 
    + I_BODY*RADIUS_WHEEL*RADIUS_WHEEL)*(M_BODY*RADIUS_WHEEL*RADIUS_WHEEL + 2*I_WHEEL)) + (RADIUS_WHEEL*RADIUS_WHEEL*(- M_BODY*DIST_WHEELS*DIST_WHEELS 
    + I_BODY)*(DisturbanceTemp[0] + TorqueInput[0] + (M_BODY*RADIUS_WHEEL*gravity*cos(phi_ref)*sin(theta))/2 + (D_MASSCENTER*RADIUS_WHEEL*RADIUS_WHEEL*gz*_rpm[1]/60.0*2*PI
      *(M_BODY - 2*M_WHEEL))/(2*DIST_WHEELS) + (D_MASSCENTER*M_BODY*RADIUS_WHEEL*gravity*sin(phi_ref)*sin(theta))/(2*DIST_WHEELS)))/
    ((2*I_WHEEL*DIST_WHEELS*DIST_WHEELS + I_BODY*RADIUS_WHEEL*RADIUS_WHEEL)*(M_BODY*RADIUS_WHEEL*RADIUS_WHEEL + 2*I_WHEEL));


  /*uk[0] = ((M_BODY*DIST_WHEELS*DIST_WHEELS*RADIUS_WHEEL*RADIUS_WHEEL + 4*I_WHEEL*DIST_WHEELS*DIST_WHEELS + I_BODY*RADIUS_WHEEL*RADIUS_WHEEL)
    *((D_MASSCENTER*gz*_rpm[1]*(M_BODY - 2*M_WHEEL)*RADIUS_WHEEL*RADIUS_WHEEL)/(2*DIST_WHEELS) + (M_BODY*gravity*cos(phi_ref)*sin(theta)*RADIUS_WHEEL)/2 + DisturbanceTemp[0]+ TorqueInput[0]))
  /((2*I_WHEEL*DIST_WHEELS*DIST_WHEELS + I_BODY*RADIUS_WHEEL*RADIUS_WHEEL)*(M_BODY*RADIUS_WHEEL*RADIUS_WHEEL + 2*I_WHEEL)) 
  + (RADIUS_WHEEL*RADIUS_WHEEL*(- M_BODY*DIST_WHEELS*DIST_WHEELS + I_BODY)*(DisturbanceTemp[1] + TorqueInput[1] + (M_BODY*RADIUS_WHEEL*gravity*cos(phi_ref)*sin(theta))/2 
    - (D_MASSCENTER*RADIUS_WHEEL*RADIUS_WHEEL*gz*_rpm[0]*(M_BODY - 2*M_WHEEL))/(2*DIST_WHEELS)))/((2*I_WHEEL*DIST_WHEELS*DIST_WHEELS + I_BODY*RADIUS_WHEEL*RADIUS_WHEEL)*(M_BODY*RADIUS_WHEEL*RADIUS_WHEEL + 2*I_WHEEL));   
*/
  uk[0] = ((M_BODY*DIST_WHEELS*DIST_WHEELS*RADIUS_WHEEL*RADIUS_WHEEL + 4*I_WHEEL*DIST_WHEELS*DIST_WHEELS + I_BODY*RADIUS_WHEEL*RADIUS_WHEEL)
    *(DisturbanceTemp[0] + TorqueInput[0] + (M_BODY*RADIUS_WHEEL*gravity*cos(phi_ref)*sin(theta))/2 + (D_MASSCENTER*RADIUS_WHEEL*RADIUS_WHEEL*gz*_rpm[1]
      *(M_BODY - 2*M_WHEEL))/(2*DIST_WHEELS) + (D_MASSCENTER*M_BODY*RADIUS_WHEEL*gravity*sin(phi_ref)*sin(theta))/(2*DIST_WHEELS)))/
  ((2*I_WHEEL*DIST_WHEELS*DIST_WHEELS + I_BODY*RADIUS_WHEEL*RADIUS_WHEEL)*(M_BODY*RADIUS_WHEEL*RADIUS_WHEEL + 2*I_WHEEL)) + (RADIUS_WHEEL*RADIUS_WHEEL
    *(- M_BODY*DIST_WHEELS*DIST_WHEELS + I_BODY)*(DisturbanceTemp[1] + TorqueInput[1] + (M_BODY*RADIUS_WHEEL*gravity*cos(phi_ref)*sin(theta))/2 
    - (D_MASSCENTER*RADIUS_WHEEL*RADIUS_WHEEL*gz*_rpm[0]*(M_BODY - 2*M_WHEEL))/(2*DIST_WHEELS) - (D_MASSCENTER*M_BODY*RADIUS_WHEEL*gravity*sin(phi_ref)
    *sin(theta))/(2*DIST_WHEELS)))/((2*I_WHEEL*DIST_WHEELS*DIST_WHEELS + I_BODY*RADIUS_WHEEL*RADIUS_WHEEL)*(M_BODY*RADIUS_WHEEL*RADIUS_WHEEL + 2*I_WHEEL));

  for(int i=0; i<2; i++){
    yk[i] = angle_acc[i]; 
  }
  // K = Q*(Q+R)^-1
  // Kalman Gain Update
  float QR[2][2];
  float (*iQR)[2];

  for(int i = 0 ; i<2 ; i++){
    for(int j =0; j<2 ; j++){
      QR[i][j] = Qk[i][j]+Rk[i][j];
    }
  }

  iQR = InverseMatrix2(QR); // if not compile, change it
  
  for(int i = 0 ; i<2 ; i++){
    for(int j =0; j<2 ; j++){
      for(int k = 0; k<2; k++){
        Kalman_Gain[i][j] += Qk[i][k]*iQR[k][j];
      }
    }
  }
  // Kalman measurement update 
  xk[1] = uk[1] + Kalman_Gain[0][0]*(yk[1]-uk[1])+Kalman_Gain[0][1]*(yk[0]-uk[0]);
  xk[0] = uk[0] + Kalman_Gain[1][0]*(yk[1]-uk[1])+Kalman_Gain[1][1]*(yk[0]-uk[0]);  
  angle_acc[1] = xk[1];
  angle_acc[0] = xk[0];
  DisturbanceTemp = SlopeControlDob(phi_ref, theta, _rpm, _enc_pos, xk); // secondly apply DOB
  return DisturbanceTemp;
}

float* Wheelchair::SlopeControlDob(float phi_ref, float theta, int32_t _rpm[],int32_t _enc_pos[], float angle_acc[]){
  float TotalTorque[2];
  static float Disturbance[2];
  float TorqueInput[2];
  float *TotalT;

  // first row, 1 = right, 0 = left
  TotalTorque[1] =
  (I_WHEEL+RADIUS_WHEEL*RADIUS_WHEEL/4/DIST_WHEELS*DIST_WHEELS*(M_BODY*DIST_WHEELS*DIST_WHEELS+I_BODY))*angle_acc[1]+RADIUS_WHEEL*RADIUS_WHEEL/4/DIST_WHEELS*DIST_WHEELS*(M_BODY*DIST_WHEELS*DIST_WHEELS+I_BODY)*angle_acc[0]
  + RADIUS_WHEEL*RADIUS_WHEEL/2/DIST_WHEELS*(M_BODY-2*M_WHEEL)*D_MASSCENTER*gz/180.0*PI*_rpm[0]/60.0*2*PI + 1/2*(-M_BODY*gravity)*cos(phi_ref)*sin(theta)*RADIUS_WHEEL+1/2*D_MASSCENTER/DIST_WHEELS*M_BODY*gravity*sin(phi_ref)*sin(theta)*RADIUS_WHEEL;
  // second row,
  TotalTorque[0] =
  (I_WHEEL+RADIUS_WHEEL*RADIUS_WHEEL/4/DIST_WHEELS*DIST_WHEELS*(M_BODY*DIST_WHEELS*DIST_WHEELS+I_BODY))*angle_acc[0]+RADIUS_WHEEL*RADIUS_WHEEL/4/DIST_WHEELS*DIST_WHEELS*(M_BODY*DIST_WHEELS*DIST_WHEELS+I_BODY)*angle_acc[1]
  - RADIUS_WHEEL*RADIUS_WHEEL/2/DIST_WHEELS*(M_BODY-2*M_WHEEL)*D_MASSCENTER*gz/180.0*PI*_rpm[1]/60.0*2*PI + 1/2*(-M_BODY*gravity)*cos(phi_ref)*sin(theta)*RADIUS_WHEEL-1/2*D_MASSCENTER/DIST_WHEELS*M_BODY*gravity*sin(phi_ref)*sin(theta)*RADIUS_WHEEL;
  TotalT = lpf_dob(TotalTorque, 0.1, 0.015);
  for (int i =0; i < 2; i++){
    TotalTorque[i] = TotalT[i]; // if not work, use for loop
    TorqueInput[i] = loadcell_current[i]*K_t;
    Disturbance[i] = TotalTorque[i]-TorqueInput[i];
  }  
  return Disturbance;
}

float (*MatMultiply_AB(float a[][2], float b[])){
  static float ResultMat[3];
  for(int i = 0; i++; i<3){
    for(int j = 0; j++; j<2){
      ResultMat[i] = a[i][j]*b[j];
    }
  }
  return ResultMat;
}
float (*MatMultiply_AtA(float a[][3]))[3]{
  static float ResultMat[3][3];
  for(int i = 0; i++; i<3){
    for(int j = 0; j++; j<3){
      for(int k = 0; k++; k<2){
        ResultMat[i][j] += a[k][i]*a[k][j];
      }
    }
  }
  return ResultMat;
}
float (*MatMultiply_AB_total(float a[][3], float b[])){
  static float ResultMat[3];
  for(int i = 0; i++; i<3){
    for(int j = 0; j++; j<3){
      ResultMat[i] = a[i][j]*b[j];
    }
  }
  return ResultMat;
}
float (*MatMultiply32(float a[][2], float b[][3]))[3]{
  static float ResultMat[3][3];
  for(int i = 0 ; i++; i<3){
    for(int j =0 ; j++; j<3){
      for(int k =0; k++; k<2){
        ResultMat[i][j] += a[i][k]*b[k][j];
      }
    }
  }
  return ResultMat;
}

float* Wheelchair::ParameterEstimator(float angle_acc[], int32_t _rpm[], float loadcell_current[]){
  static float *Parameter;
  static float AtA[3][3]={{0,},{0,},{0,}};
  static float AtB[3] = {0,};
  float *pAtB; float (*pAtA)[3]; float (*pinvAtA)[3]; float (*pinvAtA_)[3];
  float A[2][3]; static float A_accum[2][3] = {{0,},{0,}};
  float (*TA)[2]; static float (*TA_accum)[2];
  float TorqueInput[2]; static float TorqueInput_accum[2] = {0,0};
  float AtA_[3][3]; float AtB_[3];

  for(int i = 0; i++; i<2){
    TorqueInput[i] = loadcell_current[i]*K_t;
  }
  A[0][0] = angle_acc[1]; A[0][1] = angle_acc[0];  A[0][2] = _rpm[0]/60.0*2*PI*gz;
  A[1][0] = angle_acc[0]; A[1][1] = angle_acc[1];  A[1][2] = -_rpm[1]/60.0*2*PI*gz;
  TA = Transpose32(A);
  pAtB = MatMultiply_AB(TA, TorqueInput); // 3 by 1 column
  for(int i = 0; i++; i<3){
    AtB[i] += pAtB[i];
  }
  pAtA = MatMultiply_AtA(A);
  for(int i = 0; i++; i<3){
    for(int j = 0; j++; j<3){
      AtA[i][j] += pAtA[i][j];
    }
  }
  pinvAtA = InverseMatrix3(AtA);
  // B = AX linear regression
  Parameter = MatMultiply_AB_total(pinvAtA, AtB);
  // B = AX + C linear regression
  /*
  for(int i = 0; i++; i<2){
    for(int j =0; j++; j<3){
      A_accum[i][j] += A[i][j];
    }
  }
  TA_accum = Transpose32(A_accum);
  for(int i = 0; i++;i<2){
    TorqueInput_accum[i] += TorqueInput[i];
  }
  for(int i = 0; i++; i<3){
    AtB_[i] = AtB[i] - MatMultiply_AB(TA_accum, TorqueInput_accum)[i];
    for(int j = 0; j++; j<3){
      AtA_[i][j] = AtA[i][j] - MatMultiply32(TA_accum, A_accum)[i][j];      
    }
  }
  pinvAtA_ = InverseMatrix3(AtA_);
  Parameter = MatMultiply_AB_total(pinvAtA_, AtB_);
  */
  return Parameter;
}

float* Wheelchair::DesiredMotion(float loadcell_current[]){
  float TorqueInput[2];
  TorqueInput[1] = K_t*loadcell_current[1];
  TorqueInput[0] = K_t*loadcell_current[0];
  // from MATLAB, M^-1 * TAU
  DesiredAcc[1] = 
  (TorqueInput[1]*(M_BODY_d*DIST_WHEELS*DIST_WHEELS*RADIUS_WHEEL*RADIUS_WHEEL + 4*I_WHEEL*DIST_WHEELS*DIST_WHEELS 
    + I_BODY_d*RADIUS_WHEEL*RADIUS_WHEEL))  /(4*DIST_WHEELS*DIST_WHEELS*I_WHEEL*I_WHEEL + 2*M_BODY_d*DIST_WHEELS
  *DIST_WHEELS*I_WHEEL*RADIUS_WHEEL*RADIUS_WHEEL + 2*I_BODY_d*I_WHEEL*RADIUS_WHEEL*RADIUS_WHEEL 
  + I_BODY_d*M_BODY_d*pow(RADIUS_WHEEL,4)) + (RADIUS_WHEEL*RADIUS_WHEEL*TorqueInput[0]*(- M_BODY_d*DIST_WHEELS*DIST_WHEELS 
    + I_BODY_d))/(4*DIST_WHEELS*DIST_WHEELS*I_WHEEL*I_WHEEL + 2*M_BODY_d*DIST_WHEELS*DIST_WHEELS*I_WHEEL*RADIUS_WHEEL
  *RADIUS_WHEEL + 2*I_BODY_d*I_WHEEL*RADIUS_WHEEL*RADIUS_WHEEL + I_BODY_d*M_BODY_d*pow(RADIUS_WHEEL,4));
  DesiredAcc[0] = 
 (TorqueInput[0]*(M_BODY_d*DIST_WHEELS*DIST_WHEELS*RADIUS_WHEEL*RADIUS_WHEEL + 4*I_WHEEL*DIST_WHEELS*DIST_WHEELS 
  + I_BODY_d*RADIUS_WHEEL*RADIUS_WHEEL))/(4*DIST_WHEELS*DIST_WHEELS*I_WHEEL*I_WHEEL + 2*M_BODY_d*DIST_WHEELS*DIST_WHEELS*I_WHEEL*RADIUS_WHEEL*RADIUS_WHEEL 
 + 2*I_BODY_d*I_WHEEL*RADIUS_WHEEL*RADIUS_WHEEL + I_BODY_d*M_BODY_d*pow(RADIUS_WHEEL,4)) + (RADIUS_WHEEL*RADIUS_WHEEL*TorqueInput[1]
 *(- M_BODY_d*DIST_WHEELS*DIST_WHEELS + I_BODY_d))/(4*DIST_WHEELS*DIST_WHEELS*I_WHEEL*I_WHEEL + 2*M_BODY_d*DIST_WHEELS
 *DIST_WHEELS*I_WHEEL*RADIUS_WHEEL*RADIUS_WHEEL + 2*I_BODY_d*I_WHEEL*RADIUS_WHEEL*RADIUS_WHEEL + I_BODY_d*M_BODY_d*pow(RADIUS_WHEEL,4));
}

void Wheelchair::brake_position() {
  _pid_control();
}
void Wheelchair::brake_rpm() {
  _vesc_1->set_rpm(0);
  _vesc_2->set_rpm(0);
}


led_state_e Wheelchair::loadcell_initialize() {
  int16_t crc[LOADCELL_COUNT] = {0,};
  int16_t read_val[LOADCELL_COUNT] = {0,};
  int16_t crcval;
  unsigned char val_in[LOADCELL_COUNT][2] = {{0,}};
  unsigned char crc_in[LOADCELL_COUNT][2] = {{0,}};

  if (flash_read_u8(FIRST_CHECK) == 0xFF) {
    return FACTORY;
  }
  else {
    /*
    read_val[LF] = flash_read_16(LF_H);
    crc[LF] = flash_read_16(CRC_LF_H);
    read_val[RF] = flash_read_16(RF_H);
    crc[RF] = flash_read_16(CRC_RF_H);
    */
    read_val[LB] = flash_read_16(LB_H);
    crc[LB] = flash_read_16(CRC_LB_H);
    read_val[RB] = flash_read_16(RB_H);
    crc[RB] = flash_read_16(CRC_RB_H);

    for (int i = 2; i < LOADCELL_COUNT; i++)
    {
      val_in[i][1] = util_hi_byte(read_val[i]);
      val_in[i][0] = util_lo_byte(read_val[i]);
      crcval = util_update_crc(val_in[i], 2);
      if (crcval != crc[i]) return ERROR_CRC;
      else loadcell_offset[i] = read_val[i]; // restore the saved offset data
    }
  }

  return START_UP;
}

void Wheelchair::set_LED(led_state_e state) {
  _all_led_off();

  switch (state) {
    case VBAT:
      if (this->battery_volt >= 24.8) {
        _all_led_on();

      }
      else if (this->battery_volt >= 23.8 && this->battery_volt <24.8) {
        _led1_on();
        _led2_on();
        _led3_on();
        _led4_off();
      }
      else if (this->battery_volt >= 22.6 && this->battery_volt < 23.8) {
        _led1_on();
        _led2_on();
        _led3_off();
        _led4_off();
      }
      else if (this->battery_volt < 22.6) {
        _led1_on();
        _led2_off();
        _led3_off();
        _led4_off();
      }
      break;
    case CHG:		 // <4-3-2-1>
      _led4_on();
      delay(250);
      _led3_on();
      delay(250);
      _led2_on();
      delay(250);
      _led1_on();
      delay(250);
      break;
    case CALIBRATION: // <43 - off - 21 - off>
      for (int i = 0; i < 4; i++) {
        _all_led_off();
        _alert_led_on();
        _led4_on();
        _led3_on();
        delay(250);
        _all_led_off();
        _led2_on();
        _led1_on();
        delay(250);
      }
      break;
    case START_UP:	 // <4-3-2-1>
      _led4_on();
      delay(100);
      _led3_on();
      delay(100);
      _led2_on();
      delay(100);
      _led1_on();
      delay(100);
      break;
    case FACTORY:	 // <A41 - A23>
      _all_led_off();
      _alert_led_on();
      _led4_on();
      _led1_on();
      delay(200);
      _alert_led_on();
      _led4_off();
      _led1_off();
      _led2_on();
      _led3_on();
      delay(200);
      _alert_led_on();
      _led2_off();
      _led3_off();
      break;
    case ERROR_UV:	 // <A1>
      _all_led_off();
      _alert_led_on();
      _led1_on();
      break;
    case ERROR_OV:	 // <A2>
      _all_led_off();
      _alert_led_on();
      _led2_on();
      break;
    case ERROR_CRC:	 // <A3>
      _all_led_off();
      _alert_led_on();
      _led3_on();
      delay(200);
      break;
  }
}

void Wheelchair::auto_calibration() {
  if (flag_cal == LOW) {
    cal_idx = 0;
    for (int i = 2; i < LOADCELL_COUNT; i++) {
      loadcell_value_raw_calibration[i][cal_idx] = loadcell_value_raw[i];
    }
    flag_cal = HIGH;
  }
  else if (flag_cal == HIGH) {
    for (int i = 2; i < LOADCELL_COUNT; i++) {
    }
    if (flag_cal == HIGH) {
      cal_idx++;
      if (cal_idx >= AUTO_CALIBRATION_LIMIT) {
        _alert_led_on();
        for (int i = 2; i < LOADCELL_COUNT; i++) {
          loadcell_offset[i] = util_average_window(loadcell_value_raw_calibration[i], 0, 10) + loadcell_offset[i];
          // if difference below 2, calibration is saved and average them and add loadcell_offset
        }
        flash_loadcell(loadcell_offset);
        _alert_led_off();
        flag_cal = LOW;
      }
    }
  }
}






































































































































































































































































