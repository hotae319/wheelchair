#include "Wheelchair.h"
#include "flash.h"
#include "MPU9250_wheelchair.h"

int _touch_count = 0;
uint32_t stop_t;
bool straight_flag;
float smoothing(float data, float tau, float ts){
  static float pre_value = 0; float lpf_value1;
  if(straight_flag){
    pre_value = 0;
  }
  lpf_value1 = (tau * pre_value + ts * data) / (tau + ts);
  pre_value = lpf_value1;
  return lpf_value1;
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

void Wheelchair::setup() {

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
        brake_timer = millis();
        break;

      case BRAKE_3:
        _vesc_on();
        _pid_reset();
        stop_t = millis();
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

        _vesc_1->set_current(loadcell_current[1]+right_current);
        _vesc_2->set_current(loadcell_current[0]-left_current);

        if(print_flag){
          Serial.print("left_current, right_current:");
          Serial.print(left_current);
          Serial.print(",");
          Serial.println(right_current);
        }
        break;
      }   

      case BRAKE_1:
        set_LED(VBAT);
        _vesc_1->set_current_brake(10);
        _vesc_2->set_current_brake(10);
        break;

      case BRAKE_2:
        set_LED(VBAT);
        _vesc_1->set_current_brake(25);
        _vesc_2->set_current_brake(25);
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
          // smoothing the skyrocketed value
          float smooth_current1 = smoothing(loadcell_current[1], 0.1, 0.015); // have to make smoothing ftn.
          float smooth_current2 = smoothing(loadcell_current[0], 0.1, 0.015);
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
        } else if (time_out > 1000) {
          next_state = BRAKE_3;
        }
      case BRAKE_3:
        if (is_touched()) {
          next_state = RUN;
        } else if (time_out > 8000) {
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

  // two states(digital) battery(analog)
  _get_power_state();
  _get_battery_v();
  _get_button_state();  
  _get_imu_state(); // get imu data, store the data on the global variable

  _vesc_1->get_values();
  _vesc_2->get_values();

  t_now = millis();  
  for (int i = 0; i < 2; i++) {
    this->_current[i] 	= _vesc_ctrl->current[i+1];
    pre_rpm[i] = this->_rpm[i];
    pre_enc_pos[i] = this->_enc_pos[i];
    this->_rpm[i] = _vesc_ctrl->erpm[i+1]/POLE_PAIR;
    this->_enc_pos[i] = _vesc_ctrl->enc_pos[i+1];
    if( _rpm[i] != pre_rpm[i]){
      this->_accel[i] = float((this->_rpm[i] - pre_rpm[i])/(t_now-t_pre)*1000/60*2*PI);
      Serial.print("accel, current, t_now, t_pre:");
      Serial.print(_accel[i]);
      Serial.print(",");
      Serial.print(loadcell_current[i]);
      Serial.print(",");
      Serial.print(t_now);
      Serial.print(",");
      Serial.println(t_pre);
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
  // pose estimation update
  this->_cart_pose.inclination = _imu_pose._theta; // from imu data 
  //this->_cart_pose.rotation = float((this->_enc_pos[0]-this->_enc_pos[1])/DIST_WHEELS)*W_RADIUS; //
  float err1 = _enc_pos[1]-pre_enc_pos[1];
  float err2 = _enc_pos[0]-pre_enc_pos[0];  
  if((_enc_pos[1]-pre_enc_pos[1])*_rpm[1] < 0 && abs(_enc_pos[1]-pre_enc_pos[1])>200){
    err1 -= 360*sgn(_enc_pos[1]-pre_enc_pos[1]);
  }
  else if((_enc_pos[0]-pre_enc_pos[0])*_rpm[0] < 0 && abs(_enc_pos[0]-pre_enc_pos[0])>200){
    err2 -= 360*sgn(_enc_pos[0]-pre_enc_pos[0]);
  }
  odometry = (err1+err2)/DIST_WHEELS*W_RADIUS;
   
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
  // this->_cart_pose.rotation += ((_rpm[0]-pre_rpm[0])-(_rpm[1]-pre_rpm[1]))/DIST_WHEELS*W_RADIUS*(t_now-t_pre);
  t_pre = millis();*/
  //this->_cart_pose.rotation = this->_cart_pose.rotation - int(this->_cart_pose.rotation/360)*360;
  this->loadcell_update();

  if(is_touched() && j<25 && abs(_accel[1])>0.5){
    current_gain = ((j-1)*current_gain + loadcell_current[1]/10 * 4/_accel[1] * (LOADCELL_CURRENT_GAIN_B+1)-1)/j;
    j++;
    Serial.print("j:");
    Serial.print(j);
    Serial.print(",");
    Serial.print("current_gain:");
    Serial.println(current_gain);
  }

}

void Wheelchair::_get_imu_state() {
  mpu9250_wheelchair_act(); 
  //_imu_pose._roll = get_roll();
  //_imu_pose._pitch = get_pitch();
  //_imu_pose._yaw = get_yaw();  
  _imu_pose._theta = get_theta();
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
    if(err*sgn(_rpm[i])<0){
      err += 360*sgn(_rpm[i]);
    }
    /*
    if(abs(err) >300){
      err -= 360*sgn(err);
    } */   


    float err_p = err * Kp;
    float err_d = Kd * (err - err_prev[i]) / (millis()-pid_start_t) *1000; // delta_t = 0.015

    if(print_flag){
      Serial.print("err, err_d:");
      Serial.print(err);  
      Serial.print(",");    
      Serial.println(err_d);
    }

    err_i[i] += err * Ki * 0.015;
    pid_out[i] = err_p + err_i[i] + err_d;
    err_prev[i] = err;
    pid_start_t = millis();
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
  *left = (d/DIST_WHEELS*sin(phi_ref)-1/2*cos(phi_ref))*m*g*sin(theta)*W_RADIUS/K_t;
  *right = -(d/DIST_WHEELS*sin(phi_ref)+1/2*cos(phi_ref))*m*g*sin(theta)*W_RADIUS/K_t;
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
    *left_correction = smoothing(-(kp_s*err_yaw+kd_s*gz),0.1,0.015);
    *right_correction = *left_correction;
  }
  else{
    *left_correction = 0;
    *right_correction = 0;
  }

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
      if (this->battery_volt >= 12.5) {
        _all_led_on();

      }
      else if (this->battery_volt >= 11.5 && this->battery_volt < 12.5) {
        _led1_on();
        _led2_on();
        _led3_on();
        _led4_off();
      }
      else if (this->battery_volt >= 11.0 && this->battery_volt < 11.5) {
        _led1_on();
        _led2_on();
        _led3_off();
        _led4_off();
      }
      else if (this->battery_volt < 11.0) {
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



















































































































































































