#include "Wheelchair.h"
#include "flash.h"
#include "MPU9250_wheelchair.h"

#define PIN            12

int _touch_count = 0;
uint32_t stop_t;
float lpf_emer(float data, float tau, float ts){
  static float pre_value = 0; float lpf_value1;
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

  pinMode(HIGH_LIMIT_LEFT, INPUT);       //JW
  pinMode(HIGH_LIMIT_RIGHT, INPUT);       //JW
  pinMode(FOOT_PEDAL, INPUT);       //JW
  pinMode(LINEAR_ACTUATOR_DIRECTION, INPUT);       //JW
  pinMode(LOW_LIMIT, INPUT);       //JW
  pinMode(TIPPING_LEVER, INPUT);       //JW
  pinMode(DC_PWM, OUTPUT);       //JW
  pinMode(DC_DIR, OUTPUT);       //JW

  pinMode(TOUCH_L, INPUT);       //JW
  pinMode(TOUCH_M, INPUT);       //JW
  pinMode(TOUCH_R, INPUT);       //JW

  pinMode(LED_L, OUTPUT);
  pinMode(LED_M, OUTPUT);
  pinMode(LED_R, OUTPUT);
  
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

}

void Wheelchair::run() {
  static int emer = 0; // emergency is divided into obstacle(0)/bump(1)
  static int run_count = 0;
  static int stop_count = 0;
  static uint32_t t_pre = 0; uint32_t t_gap;
  t_gap = millis() - t_pre;
  t_pre = millis();
  int i; static int est_count = 0;
  int dummy;
  analogRead(TOUCH_R);
  analogRead(TOUCH_R);
  analogRead(TOUCH_L);
  analogRead(TOUCH_L);
  analogRead(TOUCH_M);
  analogRead(TOUCH_M);
  /*
  Serial.print("touch RLM:");
  Serial.print(analogRead(TOUCH_R));
  Serial.print(",");
  Serial.print(analogRead(TOUCH_L));
  Serial.print(",");
  Serial.println(analogRead(TOUCH_M));
*/
   
  /*
  Serial.print(loadcell_current_hand[1]);
  Serial.print(",");
  Serial.print(loadcell_current_hand[0]);
  Serial.print(",");
  Serial.print(loadcell_current[1]);
  Serial.print(",");
  Serial.print(loadcell_current[0]);
  Serial.print(",");
  Serial.print(analogRead(TOUCH_L));
  Serial.print(",");
  Serial.print(state);
  Serial.print(",");
  Serial.println(analogRead(TOUCH_M));
  */

  if(print_flag){  
    /*
    Serial.print("state: ");
    Serial.println(state);        
    Serial.print("currentR, currentL, vesc_currentR, vesc_currentL, encR, encL, rotation, inclination: ");
    Serial.print(loadcell_current[1]); // RIGHT (front +)
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
    Serial.println(_cart_pose.inclination*180.0/PI);
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
    Serial.print(loadcell_current[0]*500/LOADCELL_CURRENT_GAIN_B*0.4535/7050*0.182*9.81);
    Serial.println("N");
    Serial.print("voltage");
    Serial.println(analogRead(A8));
    */
    
    //print_flag = false;

  }
  // state check

  _update_states();
  
  Serial.print(revised_current[1]);
  Serial.print(",");
  Serial.print(revised_current[0]);
  Serial.print(",");
  Serial.print(loadcell_current_hand[1]);
  Serial.print(",");
  Serial.print(loadcell_current_hand[0]);
  Serial.print(",");
  Serial.print(_accel[1]);
  Serial.print(",");
  Serial.print(_accel[0]);
  Serial.print(",");
  Serial.print(_rpm[1]);
  Serial.print(",");
  Serial.print(_rpm[0]);
  Serial.print(",");
  Serial.print(DesiredAcc[1]);
  Serial.print(",");
  Serial.print(DesiredAcc[0]);
  Serial.print(",");
  Serial.print(DesiredSpd[1]);
  Serial.print(",");
  Serial.print(DesiredSpd[0]);
  Serial.print(",");
  Serial.print(Ctorque[1]);
  Serial.print(",");
  Serial.print(Ctorque[0]);
  Serial.print(",");
  Serial.print(pidtorque[1]);
  Serial.print(",");
  Serial.print(pidtorque[0]);
  Serial.print(",");
  Serial.print(midParam[0]);
  Serial.print(",");
  Serial.print(midParam[1]);
  Serial.print(",");
  Serial.print(midParam[2]);
  Serial.print(",");
  Serial.print(1000.0*ax);
  Serial.print(",");
  Serial.print(1000.0*axr);
  Serial.print(",");
  Serial.print(1000.0*ay);
  Serial.print(",");
  Serial.print(1000.0*ayr);
  Serial.print(",");
  Serial.print(comp_pitch);
  Serial.print(",");
  Serial.print(comp_roll);
  Serial.print(",");
  Serial.print(comp_yaw);
  Serial.print(",");
  Serial.print(comp_phi_ref);
  Serial.print(",");
  Serial.print(comp_theta*180.0f/PI);
  Serial.print(",");
  //Serial.print(M_BODY);
  Serial.print(analogRead(TOUCH_L));
  Serial.print(",");
  //Serial.print(I_BODY);
  Serial.print(analogRead(TOUCH_M));
  Serial.print(",");
  //Serial.print(EstimatedSpd[1]);
  Serial.print(analogRead(TOUCH_R));
  Serial.print(",");
  Serial.print(state);
  Serial.print(",");
  Serial.print(t_gap);
  Serial.print(",");
  Serial.println(gz);
  

  // state action
  if (state != next_state) {
    state = next_state;
    // transition action
    switch (state) {
      case RUN:
        _vesc_on();
        brake_timer = millis();
        run_count = 0;
        DesiredSpd[1] = _rpm[1];
        DesiredSpd[0] = _rpm[0];
        EstimatedSpd[1] = _rpm[1];
        EstimatedSpd[0] = _rpm[0];
        D_MASSCENTER = 0.11;
        analogWrite(LED_M, 250);
        analogWrite(LED_L, 20);
        analogWrite(LED_R, 20);
        break;
      case BRAKE_1:
        _vesc_on();
        brake_timer = millis();
        stop_count = 0;
        analogWrite(LED_M, 20);
        analogWrite(LED_L, 20);
        analogWrite(LED_R, 20);
        break;
      case BRAKE_2:
        _vesc_on();
        _pid_reset();
        brake_timer = millis();
        stop_t = millis();
        _enc_pos_stop[0] =  _enc_pos[0];
        _enc_pos_stop[1] =  _enc_pos[1];
        break;
      case BRAKE_3:
        _vesc_on();
        _enc_pos_stop[0] =  _enc_pos[0];
        _enc_pos_stop[1] =  _enc_pos[1];
        break;
      case BRAKE_SLEEP:
        _vesc_1->set_current(0);
        _vesc_2->set_current(0);
        analogWrite(LED_M, 20);
        analogWrite(LED_L, 20);
        analogWrite(LED_R, 20);
        break;
      case EMERGENCY:
        _vesc_on();
        brake_timer = millis();
        break;
      case SIT_DOWN:
        _vesc_on();
        brake_timer = millis();
        run_count = 0;
        DesiredSpd[1] = _rpm[1];
        DesiredSpd[0] = _rpm[0];
        D_MASSCENTER = 0.21;
        analogWrite(LED_M, 250);
        analogWrite(LED_L, 20);
        analogWrite(LED_R, 250);
        break;
      case LEFT:
        _vesc_on();
        brake_timer = millis();
        analogWrite(LED_M, 20);
        analogWrite(LED_R, 20);
        analogWrite(LED_L, 250);
      case CLIMB:
        _vesc_on();
        _enc_pos_stop[0] =  _vesc_ctrl->enc_pos[0];
        _enc_pos_stop[1] =  _vesc_ctrl->enc_pos[1];
        climb_timer = millis();      //JW
        break;
      case FOLDING:
        _vesc_on();
        break;
    }
  } else {
    // loop action
    switch (state) {
      case RUN:
      {                  
        //auto_calibration();
        set_LED(VBAT);
        float left_current=0, right_current=0;
      
        if(comp_theta>5){
          //g_compensation_control(comp_phi_ref, comp_theta, &left_current, &right_current);
        }   
        aug_gain = 0.3;

        DisturbanceWheel = DynamicObserver(comp_phi_ref, comp_theta, _rpm);
        Distemp = DisturbanceObserver(comp_phi_ref, comp_theta, _rpm, _enc_pos, _accel);        
        DesiredAcc = DesiredMotion(loadcell_current, loadcell_current_hand, _rpm);
     	  Ctorque = CompensationControl(_rpm, comp_phi_ref, comp_theta, DisturbanceWheel, DesiredAcc);        

        /*
        if(loadcell_current_hand[1]>loadcell_current_pre_hand[1]+3.5 || loadcell_current_hand[0]>loadcell_current_pre_hand[0]+3.5){
          _vesc_2->set_current(0);
          _vesc_1->set_current(0);
        } 
        else{
          _vesc_2->set_current(aug_gain*loadcell_current[1]);
          _vesc_1->set_current(-aug_gain*loadcell_current[0]);
        }
        */

        //velocity control
        
        //_vesc_2->set_rpm(POLE_PAIR*DesiredSpd[1]);
        //_vesc_1->set_rpm(-POLE_PAIR*DesiredSpd[0]);
                        
        //pid tracking
        pidtorque = TrackingPid(DesiredAcc, _rpm, _accel);
        if(loadcell_current_hand[1]>loadcell_current_pre_hand[1]+12 || loadcell_current_hand[0]>loadcell_current_pre_hand[0]+12){
          _vesc_2->set_current(0);
          _vesc_1->set_current(0);
          revised_current[1] = 0;
          revised_current[0] = 0;
        }
        else{
          revised_current[1] = loadcell_current[1] + pidtorque[1]/0.63/aug_gain;
          revised_current[0] = loadcell_current[0] + pidtorque[0]/0.63/aug_gain; 
          if(abs(aug_gain*revised_current[1])>LOADCELL_CURRENT_MAX){
            revised_current[1] = LOADCELL_CURRENT_MAX*sgn(aug_gain*revised_current[1])/aug_gain;
          }
          if(abs(aug_gain*revised_current[0])>LOADCELL_CURRENT_MAX){
            revised_current[0] = LOADCELL_CURRENT_MAX*sgn(aug_gain*revised_current[0])/aug_gain;
          }
          _vesc_2->set_current(aug_gain*revised_current[1]);
          _vesc_1->set_current(-aug_gain*revised_current[0]);
        }

        //midParam = ParameterEstimator(angle_acc, _rpm, loadcell_current);
        //midParam = ParameterEstimator(pre_accel, _rpm, loadcell_current_pre);
        midParam = ParameterEstimator(pre_accel, _rpm, revised_current);
        // M_BODY = midParam[0]; I_BODY = midParam[1]; D_MASSCENTER = pmidParam[2];
        //SearchParameter(loadcell_current, loadcell_current_hand, _rpm, _accel);
        SearchParameter(revised_current, loadcell_current_hand, _rpm, _accel);
        /*
        Serial.print(loadcell_current[1]);
        Serial.print(",");
        Serial.print(loadcell_current[0]);
        Serial.print(",");
        Serial.print(loadcell_current_hand[1]);
        Serial.print(",");
        Serial.print(loadcell_current_hand[0]);
        Serial.print(",");
        Serial.print(_accel[1]);
        Serial.print(",");
        Serial.print(_accel[0]);
        Serial.print(",");
        Serial.print(_rpm[1]);
        Serial.print(",");
        Serial.print(_rpm[0]);
        Serial.print(",");
        Serial.print(DesiredAcc[1]);
        Serial.print(",");
        Serial.print(DesiredAcc[0]);
        Serial.print(",");
        Serial.print(DesiredSpd[1]);
        Serial.print(",");
        Serial.print(DesiredSpd[0]);
        Serial.print(",");
        Serial.print(Ctorque[1]);
        Serial.print(",");
        Serial.print(Ctorque[0]);
        Serial.print(",");
        Serial.print(comp_theta*180.0f/PI);
        Serial.print(",");
        Serial.print(comp_phi_ref*180.0f/PI);
        Serial.print(",");
        Serial.println(gz);
        */
        
        if(print_flag){
          /*
          Serial.print("DisturbanceWheel R,L/only DOB R,L: ");
          Serial.print(DisturbanceWheel[1]);
          Serial.print(",");
          Serial.print(DisturbanceWheel[0]);
          Serial.print(",");  
          Serial.print(Distemp[1]);
          Serial.print(",");
          Serial.println(Distemp[0]);
          Serial.print("m, I, d: ");
          Serial.print(midParam[0],6);
          Serial.print(",");
          Serial.print(midParam[1],6);
          Serial.print(",");
          Serial.println(midParam[2],6);
          Serial.print("theta:");
          Serial.println(comp_theta);
          */
          print_flag = false;
          
        }
        //delete[] DisturbanceWheel;
        //DisturbanceWheel = nullptr;
        break;        
      }
  	
      case BRAKE_1:
        set_LED(VBAT);
        if(_rpm[1]<0 && _rpm[1]<0){
          _vesc_1->set_current_brake(10);
          _vesc_2->set_current_brake(10);
        }
        else{
          _vesc_1->set_current_brake(5);
          _vesc_2->set_current_brake(5);
        }
        break;

      case BRAKE_2:
        set_LED(VBAT);
        if(_rpm[1]<0 && _rpm[1]<0){
          if(SIT_DOWNTouch()){
            _vesc_1->set_current_brake(15);
            _vesc_2->set_current_brake(15);
          }
          else{
            _vesc_1->set_current_brake(10);
            _vesc_2->set_current_brake(10);
          }
        }
        else{
          _vesc_1->set_current_brake(12);
          _vesc_2->set_current_brake(12);
        }

        break;

      case BRAKE_3:
        set_LED(VBAT);
        _vesc_1->set_pos(_enc_pos_stop[0]);
        _vesc_2->set_pos(_enc_pos_stop[1]);
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
          float smooth_current2 = lpf_emer(loadcell_current[1], 0.1, 0.015); // have to make lpf_straight ftn.
          float smooth_current1 = lpf_emer(-loadcell_current[0], 0.1, 0.015);
          _vesc_1->set_current(smooth_current1);
          _vesc_2->set_current(smooth_current2);          
        }      
        // soon after we can add impedance control in case of emergency
        break;
      case SIT_DOWN:
      {
        set_LED(VBAT);
        float left_current=0, right_current=0;
        aug_gain = 0.4;

        DisturbanceWheel = DynamicObserver(comp_phi_ref, comp_theta, _rpm);
        Distemp = DisturbanceObserver(comp_phi_ref, comp_theta, _rpm, _enc_pos, _accel);
        DesiredAcc = DesiredMotion(loadcell_current, loadcell_current_hand, _rpm);
        Ctorque = CompensationControl(_rpm, comp_phi_ref, comp_theta, DisturbanceWheel, DesiredAcc);
        pidtorque = TrackingPid(DesiredAcc, _rpm, _accel);

        //_vesc_2->set_current(aug_gain*loadcell_current[1]);
        //_vesc_1->set_current(-aug_gain*loadcell_current[0]);
        
        
        //velocity control
        //_vesc_2->set_rpm(POLE_PAIR*DesiredSpd[1]);
        //_vesc_1->set_rpm(-POLE_PAIR*DesiredSpd[0]);
        
        //pid tracking
        revised_current[1] = loadcell_current[1] + pidtorque[1]/0.63/aug_gain;
        revised_current[0] = loadcell_current[0] + pidtorque[0]/0.63/aug_gain; 
        if(abs(aug_gain*revised_current[1])>LOADCELL_CURRENT_MAX){
          revised_current[1] = LOADCELL_CURRENT_MAX*sgn(revised_current[1])/aug_gain;
        }
        if(abs(aug_gain*revised_current[0])>LOADCELL_CURRENT_MAX){
          revised_current[0] = LOADCELL_CURRENT_MAX*sgn(revised_current[0])/aug_gain;
        }
        _vesc_2->set_current(aug_gain*revised_current[1]);
        _vesc_1->set_current(-aug_gain*revised_current[0]);

        //midParam = ParameterEstimator(angle_acc, _rpm, loadcell_current);
        midParam = ParameterEstimator(pre_accel, _rpm, loadcell_current_pre);
        // M_BODY = midParam[0]; I_BODY = midParam[1]; D_MASSCENTER = pmidParam[2];
        /*
        Serial.print(loadcell_current[1]);
        Serial.print(",");
        Serial.print(loadcell_current[0]);
        Serial.print(",");
        Serial.print(loadcell_current_hand[1]);
        Serial.print(",");
        Serial.print(loadcell_current_hand[0]);
        Serial.print(",");
        Serial.print(_accel[1]);
        Serial.print(",");
        Serial.print(_accel[0]);
        Serial.print(",");
        Serial.print(_rpm[1]);
        Serial.print(",");
        Serial.print(_rpm[0]);
        Serial.print(",");
        Serial.print(DesiredAcc[1]);
        Serial.print(",");
        Serial.print(DesiredAcc[0]);
        Serial.print(",");
        Serial.print(DesiredSpd[1]);
        Serial.print(",");
        Serial.print(DesiredSpd[0]);
        Serial.print(",");
        Serial.print(Ctorque[1]);
        Serial.print(",");
        Serial.print(Ctorque[0]);
        Serial.print(",");
        Serial.print(comp_theta*180.0f/PI);
        Serial.print(",");
        Serial.print(comp_phi_ref*180.0f/PI);
        Serial.print(",");
        Serial.println(gz);
        */
        
        if(print_flag){
          /*
          Serial.print("DisturbanceWheel R,L/only DOB R,L: ");
          Serial.print(DisturbanceWheel[1]);
          Serial.print(",");
          Serial.print(DisturbanceWheel[0]);
          Serial.print(",");  
          Serial.print(Distemp[1]);
          Serial.print(",");
          Serial.println(Distemp[0]);
          Serial.print("m, I, d: ");
          Serial.print(midParam[0],6);
          Serial.print(",");
          Serial.print(midParam[1],6);
          Serial.print(",");
          Serial.println(midParam[2],6);
          Serial.print("theta:");
          Serial.println(comp_theta);
          */
          print_flag = false;          
        }
          break;
      }

      case LEFT:
          set_LED(VBAT);            
          if(SIT_DOWNTouch()){
            aug_gain = 0.7;
          }
          else{
            aug_gain = 0.3;
          }
          if((loadcell_current_hand[0]<-1 && loadcell_current_hand[1]>1)|| (loadcell_current_hand[0]>0.5 && loadcell_current_hand[0]+2<loadcell_current_hand[1])){
          //left turn
            revised_current[1] = (aug_gain+0.2)*loadcell_current[1]-aug_gain*loadcell_current[0]-HAND_GAIN/K_t*loadcell_current_hand[0];
            revised_current[0] =aug_gain*loadcell_current[0]-aug_gain*loadcell_current_hand[1];

            //revised_current[1] = (0.1*revised_current[1] + 0.015 * ((aug_gain+0.2)*loadcell_current[1]-aug_gain*loadcell_current[0]-HAND_GAIN/K_t*loadcell_current_hand[0]))/0.115;
            //revised_current[0] = (0.1*revised_current[0] + 0.015 * (aug_gain*loadcell_current[0]-aug_gain*loadcell_current_hand[1]))/0.115;
            _vesc_2->set_current(revised_current[1]);
            _vesc_1->set_current(-revised_current[0]);
          }
          else if(loadcell_current_hand[0]>loadcell_current_hand[1]+3.5){
          // turn right
            revised_current[1] = (aug_gain+0.2)*loadcell_current[1]+HAND_GAIN/K_t*loadcell_current_hand[0];
            revised_current[0] = aug_gain*loadcell_current[0];
          
            //revised_current[1] = (0.1*revised_current[1] + 0.015*((aug_gain+0.2)*loadcell_current[1]+HAND_GAIN/K_t*loadcell_current_hand[0]))/0.115;
            //revised_current[0] = (0.1*revised_current[0] + 0.015 * aug_gain*loadcell_current[0])/0.115;
            _vesc_2->set_current(revised_current[1]);
            _vesc_1->set_current(-revised_current[0]);
          } 
          else{ // straight
            revised_current[1] = (aug_gain+0.2)*loadcell_current[1]+aug_gain*loadcell_current[0]+(HAND_GAIN+0.2)/K_t*loadcell_current_hand[0];
            revised_current[0] = aug_gain*loadcell_current[0];
            _vesc_2->set_current(revised_current[1]);
            _vesc_1->set_current(-revised_current[0]);
          }
          break;
      case CLIMB:
      {
        set_LED(VBAT);
        static float test[2] = {0,0};
        static bool flag = true;
        if(millis()-climb_timer<10000){
          _vesc_2->set_rpm(31*20);
          _vesc_1->set_rpm(-31*20);
          // velocity control test
          /*
          if(flag == true){            
            test[1] += 90;
            test[0] -= 90;
            if(test[1]>3000 && test[1]<=4000){
              test[1] += 15;
              test[0] -= 15;              
            }
            else if(test[1]>4000){
              flag = false;
            }
          }
          else{
            test[1] -= 100;
            test[0] += 100;
            if(test[1]<-3000 && test[1] >=-4000){
              test[1] -= 10;
              test[0] += 10;
            }
            else if(test[1]<-4000){
              flag = true;
            }
          }
          */
          //_vesc_2->set_rpm(test[1]);
          //_vesc_1->set_rpm(test[0]);          

          /*
        _enc_pos_stop[1] += 300*0.015;
        _enc_pos_stop[0] -= 300*0.015;
        if(abs(_enc_pos_stop[1])>1000){
          _enc_pos_stop[1] -= 360*sgn(_enc_pos_stop[1]);
        }
        if(abs(_enc_pos_stop[0])>1000){
          _enc_pos_stop[0] -= 360*sgn(_enc_pos_stop[0]);
        }
        _vesc_2->set_pos(_enc_pos_stop[1]);
        _vesc_1->set_pos(_enc_pos_stop[0]);
        */
        }
        else{
          _vesc_2->set_current(0.6*loadcell_current[1]);
          _vesc_1->set_current(-0.6*loadcell_current[0]);
        }
        /*
        else if(millis()-climb_timer>=5000 && millis()-climb_timer<8000){
          _vesc_2->set_current(0);
          _vesc_1->set_current(0);
        }
        else{
          _vesc_2->set_current(0.5*loadcell_current[1]);
          _vesc_1->set_current(-0.5*loadcell_current[0]);
        }
        */
        break;
      }
      case FOLDING:
        set_LED(VBAT);
        FOLD();       //JW
        break;
      }
    }
    if (power_state == POWER_STATE_CHARGING) calibration_charge();
    if (button_state == BUTTON_STATE_PRESS) calibration_reset();

    uint32_t time_out = millis() - brake_timer;
    if(print_flag){
      /*Serial.print("timeout");
      Serial.println(time_out);     */ 
    }

    // state transition
    switch (state) {
      case EMERGENCY:
        if (abs(loadcell_current[0])-abs(loadcell_current_pre[0])<20){
          // finish the emergency          
          next_state = RUN;   
          break;     
        }
      case RUN:
        if (is_touched() && MiddleTouchWeak() && !SIT_DOWNTouch()) {
          brake_timer = millis();
          if(digitalRead(TIPPING_LEVER)){
            next_state = CLIMB;
          }
          break;
        }
        else if(digitalRead(TIPPING_LEVER)){
          next_state = CLIMB;
        }
        else if(is_touched() && MiddleTouchWeak() && SIT_DOWNTouch()){
          next_state = SIT_DOWN;   

        }
        else if(is_touched() && LeftTouch()){
          next_state = LEFT;
        }
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
        else if (time_out > 200) {
          next_state = BRAKE_1;
        }      
        break;
  
      case SIT_DOWN:       //JW
        if (is_touched() && MiddleTouchWeak() && !SIT_DOWNTouch()){
           next_state = RUN;
           break; 
        }
        else if(is_touched() && SIT_DOWNTouch() && !LeftTouch()){
          next_state = SIT_DOWN;
          brake_timer = millis();
          break;
        }
        else if(is_touched() && LeftTouch()){
          next_state = LEFT;
          brake_timer = millis();
          break;
        }
        else if (time_out > 300) {
          next_state = BRAKE_1;
          break;
        }
        break;

      case LEFT:       //JW
        if (is_touched() && MiddleTouch() && !SIT_DOWNTouch()){
           next_state = RUN;
           break; 
        }
        else if(is_touched() && MiddleTouch() && SIT_DOWNTouch()){
            next_state = SIT_DOWN;
            break;
        }
        else if(is_touched() ){
          next_state = LEFT;
          brake_timer = millis();
          break;
        }
        else if (time_out > 300) {
          next_state = BRAKE_1;
          break;
        } 
        break;
      case CLIMB:        //JW
        if(is_touched() && !SIT_DOWNTouch() && (millis()-climb_timer > 10000)){
          next_state = RUN;      
        }      
        else if(is_touched() && SIT_DOWNTouch() && millis()-climb_timer > 10000){
          next_state = SIT_DOWN;
        }
        break;

      case FOLDING:        //JW
        if (is_touched() && !SIT_DOWNTouch() && digitalRead(FOOT_PEDAL) == LOW){
         digitalWrite(DC_PWM, LOW);
         next_state = RUN;
        }
        else if(is_touched() && SIT_DOWNTouch() && digitalRead(FOOT_PEDAL) == LOW){
          digitalWrite(DC_PWM, LOW);
          next_state = SIT_DOWN;
        }
        break; 

      case BRAKE_1:
        if (is_touched() && MiddleTouch() && !SIT_DOWNTouch()) {
          next_state = RUN;
        } 
        else if (time_out > 500) {
          next_state = BRAKE_2;
        }
        else if(is_touched() && MiddleTouch() && SIT_DOWNTouch()){
          next_state = SIT_DOWN;
          brake_timer = millis();
        }
        else if(is_touched() && LeftTouch()){
          next_state = LEFT;
          brake_timer = millis();
        } 
        else if(digitalRead(TIPPING_LEVER)){       
          next_state = CLIMB;
        }
        break;
      case BRAKE_2:
        if (is_touched() && MiddleTouch() && !SIT_DOWNTouch()) {
          next_state = RUN;
        } 
        else if (time_out > 1500) {
          next_state = BRAKE_3;
        }
        else if(is_touched() && MiddleTouch() && SIT_DOWNTouch()){
          next_state = SIT_DOWN;
          brake_timer = millis();
        }
        else if(is_touched() && LeftTouch()){
          next_state = LEFT;
          brake_timer = millis();
        }
        else if(digitalRead(TIPPING_LEVER)){   
          next_state = CLIMB;        
        }
        break;
      case BRAKE_3:
        if (is_touched() && !SIT_DOWNTouch() && !LeftTouch()) {
          next_state = RUN;
        } 
        else if (time_out > 6000) {
          next_state = BRAKE_SLEEP;
        }
        else if(is_touched() && SIT_DOWNTouch() && !LeftTouch()){
          next_state = SIT_DOWN;
          brake_timer = millis();
        }
        else if(is_touched() && LeftTouch()){
          next_state = LEFT;
          brake_timer = millis();
        }
        else if(digitalRead(TIPPING_LEVER)){       
          next_state = CLIMB;
          break;
        }
        break;
      case BRAKE_SLEEP:
        if (is_touched() && !SIT_DOWNTouch() && !LeftTouch()) {
          next_state = RUN;          
        }
        else if(is_touched() && SIT_DOWNTouch() && !LeftTouch()){
          next_state = SIT_DOWN;
          brake_timer = millis();
        }
        else if(is_touched() && LeftTouch()){
          next_state = LEFT;
          brake_timer = millis();
        }
        else if(digitalRead(FOOT_PEDAL)){      
          next_state = FOLDING;
          break;
        }
        else if(digitalRead(TIPPING_LEVER)){       
          next_state = CLIMB;
          break;
        }
    }
  count++;
  count %= 3;
}

int Wheelchair::LeftTouch(){       //JW  
  if(analogRead(TOUCH_L)>220){
    return 0;
  }
  else{
    return 0;
  }
}

int Wheelchair::SIT_DOWNTouch(){       //JW
  if(analogRead(TOUCH_R)>300){
    return 1;
  }
  else{
    return 0;
  }

}

bool Wheelchair::MiddleTouch(){       //JW
  if(analogRead(TOUCH_M)>400){
    return 1;
  }
  else{
    return 0;
  }
}

bool Wheelchair::MiddleTouchWeak(){       
  if(analogRead(TOUCH_M)>250){
    return 1;
  }
  else{
    return 0;
  }
}


void Wheelchair::FOLD(){       //JW
  int32_t fold_pos[2];
  static uint8_t count = 0;
  if(digitalRead(LINEAR_ACTUATOR_DIRECTION)==1){
    // castor direction change
    if(LA_HIGH_COUNT == 0 && digitalRead(LOW_LIMIT) == 1 && count<35){
      _vesc_2->set_rpm(-31*15);
      _vesc_1->set_rpm(-31*15);
      count++;
    }
    else{
      count = 0;
      if(digitalRead(HIGH_LIMIT_LEFT)==1 || digitalRead(HIGH_LIMIT_RIGHT)==1){
        digitalWrite(DC_PWM, LOW);
        LA_LOW_COUNT=0;
      }
      else{
        LA_HIGH_COUNT=LA_HIGH_COUNT+1;
        LA_LOW_COUNT=0;
        if(LA_HIGH_COUNT==1){
          digitalWrite(DC_PWM, LOW);
          delay(100);
          analogWrite(DC_PWM, 0);
          digitalWrite(DC_DIR, HIGH);
        }
        else if(LA_HIGH_COUNT <= 255 && LA_HIGH_COUNT >= 2){
            analogWrite(DC_PWM, LA_HIGH_COUNT);
            digitalWrite(DC_DIR, HIGH);
        }
        else{
            analogWrite(DC_PWM, 255);
            digitalWrite(DC_DIR, HIGH);
        }    
      }
    }
  }      
  else if(digitalRead(LINEAR_ACTUATOR_DIRECTION)==0){
    if(digitalRead(LOW_LIMIT)==1){
      digitalWrite(DC_PWM, LOW);
      LA_HIGH_COUNT=0;
    }
    else{
      LA_HIGH_COUNT=0;
      LA_LOW_COUNT=LA_LOW_COUNT+1;
      if(LA_LOW_COUNT==1){
        digitalWrite(DC_PWM, LOW);
        delay(100);
        digitalWrite(DC_PWM, 0);
        digitalWrite(DC_DIR, LOW);
      }
      else if(LA_LOW_COUNT <= 255 && LA_LOW_COUNT >= 2){
        analogWrite(DC_PWM, LA_LOW_COUNT);
        digitalWrite(DC_DIR, LOW);
      }
      else{
        analogWrite(DC_PWM, 255);
        digitalWrite(DC_DIR, LOW);
      }
    }
  } 
}


int Wheelchair::is_touched() {
  if (abs(loadcell_current_hand[0]) >= LOADCELL_CURRENT_DEADZONE ||
      abs(loadcell_current_hand[1]) >= LOADCELL_CURRENT_DEADZONE) {
      _touch_count++;
  }
  else {
    _touch_count = 0;
  }

  if (_touch_count >= 9) {
    return 1;
  }

  else {
    return 0;
  }

}

void Wheelchair::_update_states() {    
  float pre_enc_pos[2]; 
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
    this->_current[i] 	= _vesc_ctrl->current[i];
    if(i==0){
      pre_rpm[i] = -this->_rpm[i]; 
      // cover change of sign
    }
    else{
      pre_rpm[i] = this->_rpm[i]; 
    }
    pre_accel[i] = _accel[i];
    
    pre_enc_pos[i] = this->_enc_pos[i];
    this->_rpm[i] = _vesc_ctrl->erpm[i]/POLE_PAIR;
    this->_enc_pos[i] = _vesc_ctrl->enc_pos[i];
    if( _rpm[i] != pre_rpm[i]){
      this->_accel[i] = float((this->_rpm[i] - pre_rpm[i])/(t_now-t_pre)*1000.0/60*2*PI); //rad/s^2
      // enough with angle acceleration
    }
    else{
      this->_accel[i] = 0;
      //keep t_pre
    }
    if(i==1){            
      t_pre = millis();
    }
  }

  if (j==0){
    this->_enc_pos[0] = 0;
    this->_enc_pos[1] = 0; //initialization
    current_gain = 0;
    j++;
  }

    // Change of left 0's parameter's sign

  //this->_current[0] *= -1;
  //pre_enc_pos[0] *= -1;
  this->_rpm[0] *= -1;
  //this->_enc_pos[0] *= -1;
  this->_accel[0] *= -1;
  
  _get_imu_state(); // get imu data, store the data on the global variable
  // pose estimation update

  this->_cart_pose.inclination = _imu_pose._theta; // from imu data 
  

  //this->_cart_pose.rotation = float((this->_enc_pos[0]-this->_enc_pos[1])/DIST_WHEELS)*RADIUS_WHEEL; //
  float err1 = _enc_pos[1]-pre_enc_pos[1];
  float err2 = _enc_pos[0]-pre_enc_pos[0];  
  if(abs(_enc_pos[1]-pre_enc_pos[1])>200){
    err1 -= 360*sgn(_enc_pos[1]-pre_enc_pos[1]);
  }
  if(abs(_enc_pos[0]-pre_enc_pos[0])>200){
    err2 -= 360*sgn(_enc_pos[0]-pre_enc_pos[0]);
  }
  odometry = (err1+err2)/DIST_WHEELS*RADIUS_WHEEL;
     // because  left and left wheel has different direction

  correction = 0.98*(_imu_pose._phi-_imu_pose_phi_pre) + 0.02*odometry;  
  if(_cart_pose.inclination<10){
    this->_cart_pose.rotation =_imu_pose_phi_pre + correction;
    _imu_pose_phi_pre = _imu_pose._phi;
  }
  else{
    this->_cart_pose.rotation = _imu_pose_phi_pre - correction; // minus due to direction, phi_ref
    _imu_pose_phi_pre = _imu_pose._phi_ref; 
  }
  this->loadcell_update();
  loadcell_current[0] *= -1;
  loadcell_current_hand[0] *=-1;
  
}
void Wheelchair::_update_rpm(){
  _vesc_1->get_values();
  _vesc_2->get_values();

  for (int i = 0; i < 2; i++) {
    this->_current[i]   = _vesc_ctrl->current[i];
    if(i==0){
      pre_rpm[i] = -this->_rpm[i]; 
      // cover change of sign
    }
    else{
      pre_rpm[i] = this->_rpm[i]; 
    }
    pre_accel[i] = _accel[i];
    
    this->_rpm[i] = _vesc_ctrl->erpm[i]/POLE_PAIR;
    this->_enc_pos[i] = _vesc_ctrl->enc_pos[i];
    /*
    if( _rpm[i] != pre_rpm[i]){
      this->_accel[i] = float((this->_rpm[i] - pre_rpm[i])/(millis()-t_pre)*1000.0/60*2*PI); //rad/s^2
      // enough with angle acceleration
    }
    else{
      this->_accel[i] = 0;
      //keep t_pre
    }*/
  }
  // Change of left 0's parameter's sign

  //this->_current[0] *= -1;
  //pre_enc_pos[0] *= -1;
  this->_rpm[0] *= -1;
  //this->_enc_pos[0] *= -1;
  this->_accel[0] *= -1;
  
}

void Wheelchair::_get_imu_state() {
	static uint16_t imu_count = 0; 
  mpu9250_wheelchair_act(_accel[0],_accel[1],_rpm[0],_rpm[1],state); 
  if(imu_count>200){
 	_imu_pose._theta = get_theta();  
  }
  else{
  	_imu_pose._theta = atan(sqrtf(ax*ax+ay*ay)/abs(az));
  	imu_count++;
  }
  comp_phi_ref *= PI/180.0f;
  if(SIT_DOWNTouch()){
    comp_theta = acos(cos((comp_pitch-4.5)/180.0f*PI)*cos((comp_roll-2.0)/180.0f*PI));
  }
  else{
    comp_theta = acos(cos((comp_pitch-2.5)/180.0f*PI)*cos((comp_roll-1.63)/180.0f*PI));
  }

}
void Wheelchair::loadcell_update() {
  static uint8_t vibration_count = 0;
  static short SignCount = 0;
  static uint8_t RelayCount = 0;
  static int Rcount = 7;
  static int Lcount = 7;
  static short relay[5];
  static bool change_flag = false;
  for (int i = 2; i < LOADCELL_COUNT; i++)
  {
    int16_t temp;
    temp = loadcell_read_off(i); // subtract offset
    // kg to N  2(left, minus =front), 3(right, plus = front)
    //     Reading[i] = temp;

    if (abs(loadcell_value_raw_prev[i]) <= LOADCELL_RAW_ZERO_DEADZONE) { 
      if (abs(temp - loadcell_value_raw_prev[i]) < LOADCELL_RAW_ZERO_NOISE_THRES) {
        loadcell_value_raw[i] = temp;
        loadcell_value_raw_prev[i] = loadcell_value_raw[i];
      }
    }
    else if (abs(temp - loadcell_value_raw_prev[i]) < LOADCELL_RAW_NOISE_THRES) { // it means 40 diff, no meaning
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
        loadcell_current_pre_hand[i] = loadcell_current_hand[i];
        loadcell_current_hand[i] = float(loadcell_value_lpf[i + 2]) * LOADCELL_CURRENT_GAIN_B;      
        loadcell_current[i] = float(loadcell_value_lpf[i + 2]) * LOADCELL_CURRENT_GAIN_B;      
      }
      break;
  }
  // tremble control - low vibration, low nimble
    /*
    if(abs(loadcell_current_pre[1]-loadcell_current[1])>5 || abs(loadcell_current_pre[0]+loadcell_current[0])>5){
      if(abs(loadcell_current_pre_hand[1]-loadcell_current[1])<1 || abs(loadcell_current_pre_hand[0]+loadcell_current[0])<1){
      relay[vibration_count] = sgn(loadcell_current_hand[1]-loadcell_current_pre_hand[1]);
      }
      else{
        relay[vibration_count] = 0;
      }
      loadcell_current[1] = 0.8*loadcell_current_pre[1];
      loadcell_current[0] = 0.8*loadcell_current_pre[0]; 
      vibration_count++;            
    }
    else{
        relay[vibration_count] = 0;
        vibration_count++;    
    }    
    if(vibration_count == 5){
      vibration_count =0;    
      if(abs(relay[0]+relay[1]+relay[2]+relay[3]+relay[4])==5){
        
        loadcell_current[1] = float(loadcell_value_lpf[3]) * LOADCELL_CURRENT_GAIN_B;  
        loadcell_current[0] = float(loadcell_value_lpf[2]) * LOADCELL_CURRENT_GAIN_B;   
      }  
    }
  */
  // method 2  
  /*
  if(abs(loadcell_current_pre_hand[1]-loadcell_current[1])>5 || abs(loadcell_current_pre_hand[0]+loadcell_current[0])>5){
    vibration_count++;
    loadcell_current[1] *= 0.5;
    loadcell_current[0] *= 0.5;
  }
  else{
    if(vibration_count>0){
      vibration_count--;
    }    
  }  
  if(vibration_count>3){
    loadcell_current[1] = 0;
    loadcell_current[0] = 0;
  }
  */
  //method 3 - saturation one gap maximum = 2
  /*
  if(abs(loadcell_current_pre_hand[1]-loadcell_current_hand[1])>2){    
    loadcell_current[1] = loadcell_current_pre[1] - 2*sgn(loadcell_current_pre_hand[1]-loadcell_current_hand[1]);    
  }
  if(abs(loadcell_current_pre_hand[0]+loadcell_current_hand[0])>2){
    loadcell_current[0] = -loadcell_current_pre[0] + 2*sgn(loadcell_current_pre_hand[0]+loadcell_current_hand[0]); 
  }
  if(abs(loadcell_current_pre_hand[1]-loadcell_current_hand[1])>5 || abs(loadcell_current_pre_hand[0]+loadcell_current_hand[0])>5){
    vibration_count++;
  }
  else{
    if(vibration_count>0){
      vibration_count--;
    }  
  }
  if(vibration_count>3){
    loadcell_current[1] = 0;
    loadcell_current[0] = 0;
  }*/

  //method4
  /*
  if(_rpm[1]>5){
    if(loadcell_current[1]<0 && Rcount>0){
      Rcount--;
      loadcell_current[1] = 0;
    }
    else if(loadcell_current[1]>0){
      Rcount = 7;
    }
  }
  else if(_rpm[1]<-5){
    if(loadcell_current[1]>0 && Rcount>0){
      Rcount--;
      loadcell_current[1] = 0;
    }
    else if(loadcell_current[1]<0){
      Rcount = 7;
    }
  }
  if(_rpm[0]>5){
    if(loadcell_current[0]>0 && Lcount>0){
      Lcount--;
      loadcell_current[0] = 0;
    }
    else if(loadcell_current[0]<0){
      Lcount = 7;
    }
  }
  else if(_rpm[0]<-5){
    if(loadcell_current[0]<0 && Lcount>0){
      Lcount--;
      loadcell_current[0] = 0;
    }
    else if(loadcell_current[0]>0){
      Lcount = 7;
    }
  }
*/
  
  
  if(_rpm[1]>5){
    if(loadcell_current[1]<0 || (loadcell_current[1]-loadcell_current_pre[1])<0){
      loadcell_current[1] = (0.1*loadcell_current_pre[1] + 0.015*loadcell_current_hand[1])/0.115;
    }
  }
  else if(_rpm[1]<-5){
    if(loadcell_current[1]>0 || (loadcell_current[1]-loadcell_current_pre[1])>0){
      loadcell_current[1] = (0.1*loadcell_current_pre[1] + 0.015*loadcell_current_hand[1])/0.115;
    }
  }
  if(_rpm[0]>5){
    if(loadcell_current[0]>0 || (loadcell_current[0]+loadcell_current_pre[0])>0){
      loadcell_current[0] = (0.1*-loadcell_current_pre[0] + 0.015*loadcell_current_hand[0])/0.115;
    }
  }
  else if(_rpm[0]<-5){
    if(loadcell_current[0]<0 || (loadcell_current[0]+loadcell_current_pre[0])<0){
      loadcell_current[0] = (0.1*-loadcell_current_pre[0] + 0.015*loadcell_current_hand[0])/0.115;
    }
  }
  
  if(abs(loadcell_current_pre_hand[1]-loadcell_current_hand[1])>7 || abs(loadcell_current_pre_hand[0]+loadcell_current_hand[0])>7){
    vibration_count++;
  }
  else{
    if(vibration_count>0){
      vibration_count--;
    }    
  }  
  if(vibration_count>5){
    loadcell_current[1] = 0;
    loadcell_current[0] = 0;
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

void Wheelchair::calibration_charge(){
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
    float err = _enc_pos[i] - _enc_pos_stop[i];
    if(abs(err)>240){ 
      err -= 360*sgn(err);
    }
    float err_p = err * Kp;
    float err_d = Kd * (err - err_prev[i]) / (millis()-pid_start_t) *1000.0; // delta_t = 0.015
    err_i[i] += err * Ki * 0.015;
    pid_out[i] = err_p + err_i[i] + err_d;                                                                                                                                                    
    err_prev[i] = err;
    if(i==1){
	    pid_start_t = millis();
    }

    if (pid_out[i] > PID_LIMIT)         pid_out[i] = PID_LIMIT;
    else if (pid_out[i] < -PID_LIMIT)   pid_out[i] = -PID_LIMIT;
  }
  _vesc_1->set_current(pid_out[0]);
  _vesc_2->set_current(-pid_out[1]);

}

void Wheelchair::_pid_reset() {
  memset(err_prev, 0, sizeof(err_prev));
  memset(err_i, 0, sizeof(err_i));
}

void Wheelchair::g_compensation_control(float phi_ref, float theta, float *left, float *right){
  *left = -1.0/2.0*M_BODY*gravity*sin(theta)*RADIUS_WHEEL*cos(phi_ref)*+1.0/2.0*M_BODY*gravity*sin(phi_ref)*sin(theta)*D_MASSCENTER/DIST_WHEELS*RADIUS_WHEEL;
  *right = -1.0/2.0*M_BODY*gravity*sin(theta)*RADIUS_WHEEL*cos(phi_ref)-1.0/2.0*M_BODY*gravity*sin(phi_ref)*sin(theta)*D_MASSCENTER/DIST_WHEELS*RADIUS_WHEEL;   
}


float* Wheelchair::DynamicObserver(float phi_ref, float theta, float _rpm[]){
  // It is about small state which includes only angle acceleration 
  float K1 = 0.8; float TorqueInput[2];
  //float *DisturbanceTemp = new float[2]; // dynamic allocation
  static float DisturbanceTemp[2];
  float *pDisturbanceTemp;
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
    TorqueInput[i] = loadcell_current[i]*K_t*aug_gain+HAND_GAIN*loadcell_current_hand[i];
  }
  // measurement from ax and encoder 
  angle_acc[1] = K1*_accel[1] + (1-K1)*(2.0*ax*10/RADIUS_WHEEL-_accel[0]);
  angle_acc[0] = K1*_accel[0] + (1-K1)*(2.0*ax*10/RADIUS_WHEEL-_accel[1]);  
  
  pDisturbanceTemp = DisturbanceObserver(phi_ref, theta, _rpm, _enc_pos, angle_acc); // first apply DOB
  DisturbanceTemp[1] = *(pDisturbanceTemp+1);
  DisturbanceTemp[0] = *pDisturbanceTemp;
  //Serial.print("Dis: ");
  //Serial.println(DisturbanceTemp[0]); 
  // Kalman filtering state equation, allocate uk
   uk[1] = ((M_BODY*DIST_WHEELS*DIST_WHEELS*RADIUS_WHEEL*RADIUS_WHEEL + 4*I_WHEEL*DIST_WHEELS*DIST_WHEELS + I_BODY*RADIUS_WHEEL*RADIUS_WHEEL)*(DisturbanceTemp[1] 
    + TorqueInput[1] + (M_BODY*RADIUS_WHEEL*gravity*cos(phi_ref)*sin(theta))/2 - (D_MASSCENTER*RADIUS_WHEEL*RADIUS_WHEEL*gz/180.0*PI*_rpm[0]/60.0*2*PI*(M_BODY - 2*M_WHEEL))
    /(2*DIST_WHEELS) - (D_MASSCENTER*M_BODY*RADIUS_WHEEL*gravity*sin(phi_ref)*sin(theta))/(2*DIST_WHEELS)))/((2*I_WHEEL*DIST_WHEELS*DIST_WHEELS 
    + I_BODY*RADIUS_WHEEL*RADIUS_WHEEL)*(M_BODY*RADIUS_WHEEL*RADIUS_WHEEL + 2*I_WHEEL)) + (RADIUS_WHEEL*RADIUS_WHEEL*(- M_BODY*DIST_WHEELS*DIST_WHEELS 
    + I_BODY)*(DisturbanceTemp[0] + TorqueInput[0] + (M_BODY*RADIUS_WHEEL*gravity*cos(phi_ref)*sin(theta))/2 + (D_MASSCENTER*RADIUS_WHEEL*RADIUS_WHEEL*gz/180.0*PI*_rpm[1]/60.0*2*PI
      *(M_BODY - 2*M_WHEEL))/(2*DIST_WHEELS) + (D_MASSCENTER*M_BODY*RADIUS_WHEEL*gravity*sin(phi_ref)*sin(theta))/(2*DIST_WHEELS)))/
    ((2*I_WHEEL*DIST_WHEELS*DIST_WHEELS + I_BODY*RADIUS_WHEEL*RADIUS_WHEEL)*(M_BODY*RADIUS_WHEEL*RADIUS_WHEEL + 2*I_WHEEL));

  uk[0] = ((M_BODY*DIST_WHEELS*DIST_WHEELS*RADIUS_WHEEL*RADIUS_WHEEL + 4*I_WHEEL*DIST_WHEELS*DIST_WHEELS + I_BODY*RADIUS_WHEEL*RADIUS_WHEEL)
    *(DisturbanceTemp[0] + TorqueInput[0] + (M_BODY*RADIUS_WHEEL*gravity*cos(phi_ref)*sin(theta))/2 + (D_MASSCENTER*RADIUS_WHEEL*RADIUS_WHEEL*gz/180.0*PI*_rpm[1]/60.0*2*PI
      *(M_BODY - 2*M_WHEEL))/(2*DIST_WHEELS) + (D_MASSCENTER*M_BODY*RADIUS_WHEEL*gravity*sin(phi_ref)*sin(theta))/(2*DIST_WHEELS)))/
  ((2*I_WHEEL*DIST_WHEELS*DIST_WHEELS + I_BODY*RADIUS_WHEEL*RADIUS_WHEEL)*(M_BODY*RADIUS_WHEEL*RADIUS_WHEEL + 2*I_WHEEL)) + (RADIUS_WHEEL*RADIUS_WHEEL
    *(- M_BODY*DIST_WHEELS*DIST_WHEELS + I_BODY)*(DisturbanceTemp[1] + TorqueInput[1] + (M_BODY*RADIUS_WHEEL*gravity*cos(phi_ref)*sin(theta))/2 
    - (D_MASSCENTER*RADIUS_WHEEL*RADIUS_WHEEL*gz/180.0*PI*_rpm[0]/60.0*2*PI*(M_BODY - 2*M_WHEEL))/(2*DIST_WHEELS) - (D_MASSCENTER*M_BODY*RADIUS_WHEEL*gravity*sin(phi_ref)
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
  pDisturbanceTemp = DisturbanceObserver(phi_ref, theta, _rpm, _enc_pos, xk); // secondly apply DOB
  DisturbanceTemp[1] = *(pDisturbanceTemp+1);
  DisturbanceTemp[0] = *pDisturbanceTemp;
  return DisturbanceTemp;
}

float* Wheelchair::DisturbanceObserver(float phi_ref, float theta, float _rpm[],int32_t _enc_pos[], float _angle_acc[]){
  float TotalTorque[2];
  static float Disturbance[2];
  float TorqueInput[2];
  float *TotalT;

  // first row, 1 = right, 0 = left
  TotalTorque[1] =
  (I_WHEEL+RADIUS_WHEEL*RADIUS_WHEEL/4/DIST_WHEELS*DIST_WHEELS*(M_BODY*DIST_WHEELS*DIST_WHEELS+I_BODY))*_angle_acc[1]+RADIUS_WHEEL*RADIUS_WHEEL/4/DIST_WHEELS*DIST_WHEELS*(M_BODY*DIST_WHEELS*DIST_WHEELS+I_BODY)*_angle_acc[0]
  + RADIUS_WHEEL*RADIUS_WHEEL/2/DIST_WHEELS*(M_BODY-2*M_WHEEL)*D_MASSCENTER*gz/180.0*PI*_rpm[0]/60.0*2*PI + 1/2*(-M_BODY*gravity)*cos(phi_ref)*sin(theta)*RADIUS_WHEEL+1/2*D_MASSCENTER/DIST_WHEELS*M_BODY*gravity*sin(phi_ref)*sin(theta)*RADIUS_WHEEL;
  // second row,
  TotalTorque[0] =
  (I_WHEEL+RADIUS_WHEEL*RADIUS_WHEEL/4/DIST_WHEELS*DIST_WHEELS*(M_BODY*DIST_WHEELS*DIST_WHEELS+I_BODY))*_angle_acc[0]+RADIUS_WHEEL*RADIUS_WHEEL/4/DIST_WHEELS*DIST_WHEELS*(M_BODY*DIST_WHEELS*DIST_WHEELS+I_BODY)*_angle_acc[1]
  - RADIUS_WHEEL*RADIUS_WHEEL/2/DIST_WHEELS*(M_BODY-2*M_WHEEL)*D_MASSCENTER*gz/180.0*PI*_rpm[1]/60.0*2*PI + 1/2*(-M_BODY*gravity)*cos(phi_ref)*sin(theta)*RADIUS_WHEEL-1/2*D_MASSCENTER/DIST_WHEELS*M_BODY*gravity*sin(phi_ref)*sin(theta)*RADIUS_WHEEL;
  TotalT = lpf_dob(TotalTorque, 0.1, 0.015);
  for (int i =0; i < 2; i++){
    TotalTorque[i] = TotalT[i]; // if not work, use for loop
    //Serial.print("TotalTorque:");
    //Serial.println(TotalTorque[i]);
    TorqueInput[i] = loadcell_current[i]*K_t*aug_gain+loadcell_current_hand[i]*HAND_GAIN;
    Disturbance[i] = TotalTorque[i]-TorqueInput[i];
  }  

  return Disturbance;
}


float* Wheelchair::ParameterEstimator(float _angle_acc[], float _rpm[], float loadcell_current[]){
  static float Parameter[3];
  float *X;
  static float AtA[3][3]={{0,},{0,},{0,}};
  static float AtB[3] = {0,};
  float *pAtB; float (*pAtA)[3]; float (*pinvAtA)[3]; float (*pinvAtA_)[3];
  float A[2][3]; static float A_accum[2][3] = {{0,},{0,}};
  float (*TA)[2]; static float (*TA_accum)[2];
  float TorqueInput[2]; static float TorqueInput_accum[2] = {0,0};
  float AtA_[3][3]; float AtB_[3];
  static int n = 0;
  n++;
  for(int i = 0; i<2; i++){
    TorqueInput[i] = loadcell_current[i]*K_t*aug_gain+loadcell_current_hand[i]*HAND_GAIN-(0.554+0.5)*sgn(_rpm[i])-1.0/120.0*_rpm[i];;
  }
  A[0][0] = _angle_acc[1]; A[0][1] = _angle_acc[0];  A[0][2] = _rpm[0]/60.0*2*PI*gz/180.0*PI;
  A[1][0] = _angle_acc[0]; A[1][1] = _angle_acc[1];  A[1][2] = -_rpm[1]/60.0*2*PI*gz/180.0*PI;
  TA = Transpose32(A);
  pAtB = MatMultiply_AB(TA, TorqueInput); // 3 by 1 column
  /*Serial.print("A: ");
  Serial.println(A[0][0]);
    Serial.println(A[1][0]);
      Serial.println(A[0][2]);
        Serial.println(A[1][2]);*/
  for(int i = 0; i<3; i++){
    AtB[i] += pAtB[i];
  }
  pAtA = MatMultiply_AtA(A);

  for(int i = 0; i<3; i++){
    for(int j = 0; j<3; j++){
      AtA[i][j] += pAtA[i][j];
    }
  }
  pinvAtA = InverseMatrix3(AtA);

  // B = AX linear regression
  /*
  X = MatMultiply_AB_total(pinvAtA, AtB);
  if(print_flag){
    
    Serial.print("X: ");
    Serial.println(*X,9);
    Serial.println(*(X+1),9);
    Serial.println(*(X+2),9);
    
  }
  */
  // B = AX + C linear regression
  
  for(int i = 0; i<2; i++){
    for(int j =0; j<3; j++){
      A_accum[i][j] += A[i][j];
    }
  }
  TA_accum = Transpose32(A_accum);
  for(int i = 0; i<2; i++){
    TorqueInput_accum[i] += TorqueInput[i];
  }
  for(int i = 0; i<3; i++){
    AtB_[i] = AtB[i] - 1/n*MatMultiply_AB(TA_accum, TorqueInput_accum)[i];
    for(int j = 0; j<3; j++){
      AtA_[i][j] = AtA[i][j] - 1/n*MatMultiply32(TA_accum, A_accum)[i][j];      
    }
  }
  pinvAtA_ = InverseMatrix3(AtA_);
  X = MatMultiply_AB_total(pinvAtA_, AtB_);

  if(print_flag){  
    /*
    Serial.print("X: ");
    Serial.println(*X,9);
    Serial.println(*(X+1),9);
    Serial.println(*(X+2),9);
    */
  }
  // m, I, d output
  Parameter[0] = (*(X+0)+*(X+1)-I_WHEEL)/(1.0/2.0*RADIUS_WHEEL*RADIUS_WHEEL);
  Parameter[1] = (*(X+0)-*(X+1)-I_WHEEL)/(1.0/2.0*RADIUS_WHEEL*RADIUS_WHEEL/DIST_WHEELS/DIST_WHEELS);
  Parameter[2] = 2.0*DIST_WHEELS*(*(X+2))/(*Parameter - 2.0*M_WHEEL)/RADIUS_WHEEL/RADIUS_WHEEL; 
 
  return Parameter;
}

float* Wheelchair::DesiredMotion(float loadcell_current[], float loadcell_current_hand[], float _rpm[]){
  float TorqueInput[2];
  
  static float desired_acc[2];  
  float rk1 = 0.5, rk2 = 0.5;

  //TorqueInput[1] = K_t*aug_gain*loadcell_current[1]+HAND_GAIN*loadcell_current_hand[1]-(0.654)*sgn(_rpm[1])-1.0/60.0*_rpm[1]; // with 0.02 mu friction const
  //TorqueInput[0] = K_t*aug_gain*loadcell_current[0]+HAND_GAIN*loadcell_current_hand[0]-(0.654)*sgn(_rpm[0])-1.0/60.0*_rpm[0];

  TorqueInput[1] = K_t*aug_gain*loadcell_current[1]+HAND_GAIN*loadcell_current_hand[1]-(0.454)*sgn(DesiredSpd[1])-1.0/180.0*DesiredSpd[1]; // with 0.02 mu friction const
  TorqueInput[0] = K_t*aug_gain*loadcell_current[0]+HAND_GAIN*loadcell_current_hand[0]-(0.454)*sgn(DesiredSpd[0])-1.0/180.0*DesiredSpd[0];

  if(TorqueInput[1]*loadcell_current_hand[1]<0){
    TorqueInput[1] = 0;
  }
  if(TorqueInput[0]*loadcell_current_hand[0]<0){
    TorqueInput[0] = 0;
  }

  // from MATLAB, M^-1*(tau - V*rpm) rotation term
  desired_acc[1] = ((M_BODY_d*DIST_WHEELS*DIST_WHEELS*RADIUS_WHEEL*RADIUS_WHEEL + 4*I_WHEEL*DIST_WHEELS*DIST_WHEELS + I_BODY_d*RADIUS_WHEEL*RADIUS_WHEEL)*(2*DIST_WHEELS*TorqueInput[1] - D_MASSCENTER_d*M_BODY_d*RADIUS_WHEEL*RADIUS_WHEEL*gz/180.0*PI*_rpm[0]/60.0*2*PI + 2*D_MASSCENTER_d*M_WHEEL*RADIUS_WHEEL*RADIUS_WHEEL*gz/180.0*PI*_rpm[0]/60.0*2*PI))/(2*DIST_WHEELS*(2*I_WHEEL*DIST_WHEELS*DIST_WHEELS + I_BODY_d*RADIUS_WHEEL*RADIUS_WHEEL)*(M_BODY_d*RADIUS_WHEEL*RADIUS_WHEEL + 2*I_WHEEL)) + (RADIUS_WHEEL*RADIUS_WHEEL*(- M_BODY_d*DIST_WHEELS*DIST_WHEELS + I_BODY_d)*(2*DIST_WHEELS*TorqueInput[0] + D_MASSCENTER_d*M_BODY_d*RADIUS_WHEEL*RADIUS_WHEEL*gz/180.0*PI*_rpm[1]/60.0*2*PI - 2*D_MASSCENTER_d*M_WHEEL*RADIUS_WHEEL*RADIUS_WHEEL*gz/180.0*PI*_rpm[1]/60.0*2*PI))/(2*DIST_WHEELS*(2*I_WHEEL*DIST_WHEELS*DIST_WHEELS + I_BODY_d*RADIUS_WHEEL*RADIUS_WHEEL)*(M_BODY_d*RADIUS_WHEEL*RADIUS_WHEEL + 2*I_WHEEL));
  desired_acc[0] = ((M_BODY_d*DIST_WHEELS*DIST_WHEELS*RADIUS_WHEEL*RADIUS_WHEEL + 4*I_WHEEL*DIST_WHEELS*DIST_WHEELS + I_BODY_d*RADIUS_WHEEL*RADIUS_WHEEL)*(2*DIST_WHEELS*TorqueInput[0] + D_MASSCENTER_d*M_BODY_d*RADIUS_WHEEL*RADIUS_WHEEL*gz/180.0*PI*_rpm[1]/60.0*2*PI - 2*D_MASSCENTER_d*M_WHEEL*RADIUS_WHEEL*RADIUS_WHEEL*gz/180.0*PI*_rpm[1]/60.0*2*PI))/(2*DIST_WHEELS*(2*I_WHEEL*DIST_WHEELS*DIST_WHEELS + I_BODY_d*RADIUS_WHEEL*RADIUS_WHEEL)*(M_BODY_d*RADIUS_WHEEL*RADIUS_WHEEL + 2*I_WHEEL)) + (RADIUS_WHEEL*RADIUS_WHEEL*(- M_BODY_d*DIST_WHEELS*DIST_WHEELS + I_BODY_d)*(2*DIST_WHEELS*TorqueInput[1] - D_MASSCENTER_d*M_BODY_d*RADIUS_WHEEL*RADIUS_WHEEL*gz/180.0*PI*_rpm[0]/60.0*2*PI + 2*D_MASSCENTER_d*M_WHEEL*RADIUS_WHEEL*RADIUS_WHEEL*gz/180.0*PI*_rpm[0]/60.0*2*PI))/(2*DIST_WHEELS*(2*I_WHEEL*DIST_WHEELS*DIST_WHEELS + I_BODY_d*RADIUS_WHEEL*RADIUS_WHEEL)*(M_BODY_d*RADIUS_WHEEL*RADIUS_WHEEL + 2*I_WHEEL)); 
  //desired_acc[1] = ((M_BODY_d*DIST_WHEELS*DIST_WHEELS*RADIUS_WHEEL*RADIUS_WHEEL + 4*I_WHEEL*DIST_WHEELS*DIST_WHEELS + I_BODY_d*RADIUS_WHEEL*RADIUS_WHEEL)*(2*DIST_WHEELS*TorqueInput[1] - D_MASSCENTER_d*M_BODY_d*RADIUS_WHEEL*RADIUS_WHEEL*gz/180.0*PI*DesiredSpd[0]/60.0*2*PI + 2*D_MASSCENTER_d*M_WHEEL*RADIUS_WHEEL*RADIUS_WHEEL*gz/180.0*PI*DesiredSpd[0]/60.0*2*PI))/(2*DIST_WHEELS*(2*I_WHEEL*DIST_WHEELS*DIST_WHEELS + I_BODY_d*RADIUS_WHEEL*RADIUS_WHEEL)*(M_BODY_d*RADIUS_WHEEL*RADIUS_WHEEL + 2*I_WHEEL)) + (RADIUS_WHEEL*RADIUS_WHEEL*(- M_BODY_d*DIST_WHEELS*DIST_WHEELS + I_BODY_d)*(2*DIST_WHEELS*TorqueInput[0] + D_MASSCENTER_d*M_BODY_d*RADIUS_WHEEL*RADIUS_WHEEL*gz/180.0*PI*DesiredSpd[1]/60.0*2*PI - 2*D_MASSCENTER_d*M_WHEEL*RADIUS_WHEEL*RADIUS_WHEEL*gz/180.0*PI*DesiredSpd[1]/60.0*2*PI))/(2*DIST_WHEELS*(2*I_WHEEL*DIST_WHEELS*DIST_WHEELS + I_BODY_d*RADIUS_WHEEL*RADIUS_WHEEL)*(M_BODY_d*RADIUS_WHEEL*RADIUS_WHEEL + 2*I_WHEEL));
  //desired_acc[0] = ((M_BODY_d*DIST_WHEELS*DIST_WHEELS*RADIUS_WHEEL*RADIUS_WHEEL + 4*I_WHEEL*DIST_WHEELS*DIST_WHEELS + I_BODY_d*RADIUS_WHEEL*RADIUS_WHEEL)*(2*DIST_WHEELS*TorqueInput[0] + D_MASSCENTER_d*M_BODY_d*RADIUS_WHEEL*RADIUS_WHEEL*gz/180.0*PI*DesiredSpd[1]/60.0*2*PI - 2*D_MASSCENTER_d*M_WHEEL*RADIUS_WHEEL*RADIUS_WHEEL*gz/180.0*PI*DesiredSpd[1]/60.0*2*PI))/(2*DIST_WHEELS*(2*I_WHEEL*DIST_WHEELS*DIST_WHEELS + I_BODY_d*RADIUS_WHEEL*RADIUS_WHEEL)*(M_BODY_d*RADIUS_WHEEL*RADIUS_WHEEL + 2*I_WHEEL)) + (RADIUS_WHEEL*RADIUS_WHEEL*(- M_BODY_d*DIST_WHEELS*DIST_WHEELS + I_BODY_d)*(2*DIST_WHEELS*TorqueInput[1] - D_MASSCENTER_d*M_BODY_d*RADIUS_WHEEL*RADIUS_WHEEL*gz/180.0*PI*DesiredSpd[0]/60.0*2*PI + 2*D_MASSCENTER_d*M_WHEEL*RADIUS_WHEEL*RADIUS_WHEEL*gz/180.0*PI*DesiredSpd[0]/60.0*2*PI))/(2*DIST_WHEELS*(2*I_WHEEL*DIST_WHEELS*DIST_WHEELS + I_BODY_d*RADIUS_WHEEL*RADIUS_WHEEL)*(M_BODY_d*RADIUS_WHEEL*RADIUS_WHEEL + 2*I_WHEEL)); 

  //DesiredSpd[1] = _rpm[1] + desired_acc[1]*0.018/2/PI*60.0; 
  //DesiredSpd[0] = _rpm[0] + desired_acc[0]*0.018/2/PI*60.0;   
  
  DesiredSpd[1] += desired_acc[1]*0.018/2/PI*60.0; 
  DesiredSpd[0] += desired_acc[0]*0.018/2/PI*60.0;
  if(abs(DesiredSpd[1])>VEL_MAX){
    DesiredSpd[1] = VEL_MAX*sgn(DesiredSpd[1]);
  }   
  if(abs(DesiredSpd[0])>VEL_MAX){
    DesiredSpd[0] = VEL_MAX*sgn(DesiredSpd[0]);
  } 
  
  // from Matlab, M^-1(tau-k*rpm) resistance
  //desired_acc[1] = (4*DIST_WHEELS*DIST_WHEELS*I_WHEEL*TorqueInput[1] + I_BODY*RADIUS_WHEEL*RADIUS_WHEEL*TorqueInput[0] + I_BODY*RADIUS_WHEEL*RADIUS_WHEEL*TorqueInput[1] - 4*DIST_WHEELS*DIST_WHEELS*I_WHEEL*rk1*_rpm[1]/60.0*2*PI - I_BODY*RADIUS_WHEEL*RADIUS_WHEEL*rk1*_rpm[1]/60.0*2*PI - I_BODY*RADIUS_WHEEL*RADIUS_WHEEL*rk2*_rpm[0]/60.0*2*PI - DIST_WHEELS*DIST_WHEELS*M_BODY*RADIUS_WHEEL*RADIUS_WHEEL*TorqueInput[0] + DIST_WHEELS*DIST_WHEELS*M_BODY*RADIUS_WHEEL*RADIUS_WHEEL*TorqueInput[1] - DIST_WHEELS*DIST_WHEELS*M_BODY*RADIUS_WHEEL*RADIUS_WHEEL*rk1*_rpm[1]/60.0*2*PI + DIST_WHEELS*DIST_WHEELS*M_BODY*RADIUS_WHEEL*RADIUS_WHEEL*rk2*_rpm[0]/60.0*2*PI)/((2*I_WHEEL*DIST_WHEELS*DIST_WHEELS + I_BODY*RADIUS_WHEEL*RADIUS_WHEEL)*(M_BODY*RADIUS_WHEEL*RADIUS_WHEEL + 2*I_WHEEL));
  //desired_acc[0] = (4*DIST_WHEELS*DIST_WHEELS*I_WHEEL*TorqueInput[0] + I_BODY*RADIUS_WHEEL*RADIUS_WHEEL*TorqueInput[0] + I_BODY*RADIUS_WHEEL*RADIUS_WHEEL*TorqueInput[1] - 4*DIST_WHEELS*DIST_WHEELS*I_WHEEL*rk2*_rpm[0]/60.0*2*PI - I_BODY*RADIUS_WHEEL*RADIUS_WHEEL*rk1*_rpm[1]/60.0*2*PI - I_BODY*RADIUS_WHEEL*RADIUS_WHEEL*rk2*_rpm[0]/60.0*2*PI + DIST_WHEELS*DIST_WHEELS*M_BODY*RADIUS_WHEEL*RADIUS_WHEEL*TorqueInput[0] - DIST_WHEELS*DIST_WHEELS*M_BODY*RADIUS_WHEEL*RADIUS_WHEEL*TorqueInput[1] + DIST_WHEELS*DIST_WHEELS*M_BODY*RADIUS_WHEEL*RADIUS_WHEEL*rk1*_rpm[1]/60.0*2*PI - DIST_WHEELS*DIST_WHEELS*M_BODY*RADIUS_WHEEL*RADIUS_WHEEL*rk2*_rpm[0]/60.0*2*PI)/((2*I_WHEEL*DIST_WHEELS*DIST_WHEELS + I_BODY*RADIUS_WHEEL*RADIUS_WHEEL)*(M_BODY*RADIUS_WHEEL*RADIUS_WHEEL + 2*I_WHEEL));
  
  return desired_acc;
}

float* Wheelchair::CompensationControl(float _rpm[], float phi_ref, float theta, float *DisturbanceTemp, float *desired_acc){
	static float ControlTorque[2];
  float gain = 0.1;
	float V[2][2] = {{0, RADIUS_WHEEL*RADIUS_WHEEL/(2*DIST_WHEELS)*(M_BODY-2*M_WHEEL)*D_MASSCENTER*gz/180.0*PI},{-RADIUS_WHEEL*RADIUS_WHEEL/(2*DIST_WHEELS)*(M_BODY-2*M_WHEEL)*D_MASSCENTER*gz/180.0*PI, 0}};
	float StG[2] = {-1/2*M_BODY*gravity*cos(phi_ref)*sin(theta)*RADIUS_WHEEL+M_BODY*gravity*sin(phi_ref)*sin(theta)*D_MASSCENTER/DIST_WHEELS*RADIUS_WHEEL,-1/2*M_BODY*gravity*cos(phi_ref)*sin(theta)*RADIUS_WHEEL-M_BODY*gravity*sin(phi_ref)*sin(theta)*D_MASSCENTER/DIST_WHEELS*RADIUS_WHEEL};
	float M[2][2] = {{I_WHEEL+RADIUS_WHEEL*RADIUS_WHEEL/(4*DIST_WHEELS*DIST_WHEELS)*(M_BODY*DIST_WHEELS*DIST_WHEELS+I_BODY), RADIUS_WHEEL*RADIUS_WHEEL/(4*DIST_WHEELS*DIST_WHEELS)*(M_BODY*DIST_WHEELS*DIST_WHEELS-I_BODY)},{RADIUS_WHEEL*RADIUS_WHEEL/(4*DIST_WHEELS*DIST_WHEELS)*(M_BODY*DIST_WHEELS*DIST_WHEELS-I_BODY), I_WHEEL+RADIUS_WHEEL*RADIUS_WHEEL/(4*DIST_WHEELS*DIST_WHEELS)*(M_BODY*DIST_WHEELS*DIST_WHEELS+I_BODY)}};
	// StG + V*mu + M*mu_d dot
  ControlTorque[1] = V[0][0]*_rpm[1]/60.0*2*PI + V[0][1]*_rpm[0]/60.0*2*PI + StG[0] - DisturbanceTemp[1] + M[0][0]*desired_acc[1]+M[0][1]*desired_acc[0] - gain*(_rpm[1]-DesiredSpd[1]);
	ControlTorque[0] = V[1][0]*_rpm[1]/60.0*2*PI + V[1][1]*_rpm[0]/60.0*2*PI + StG[1] - DisturbanceTemp[0] + M[1][0]*desired_acc[1]+M[1][1]*desired_acc[0] - gain*(_rpm[0]-DesiredSpd[0]);
	return ControlTorque;
}

float* Wheelchair::TrackingPid(float *desired_acc, float _rpm[], float _accel[]){
  static float ControlTorque[2];
  float p_gain = 0.1; float d_gain = 0.00;
  ControlTorque[1] = -p_gain*(_rpm[1]-DesiredSpd[1]) - d_gain*(_accel[1]-desired_acc[1]);
  ControlTorque[0] = -p_gain*(_rpm[0]-DesiredSpd[0]) - d_gain*(_accel[0]-desired_acc[0]);
  return ControlTorque;
}

void Wheelchair::SearchParameter(float loadcell_current[], float loadcell_current_hand[], float _rpm[], float _accel[]){
  float TorqueInput[2];
  float EstimatedAcc[2]; 
  static float Esum_acc = 0; static float Rsum_acc = 0; static float Esum_spd = 0; static float Rsum_spd = 0;
  /*
  static float AvgEstacc[10]; 
  static float AvgRealacc[10];
  static float AvgEstspd[10]; 
  static float AvgRealspd[10];
  */
  static uint8_t idx = 0;
  TorqueInput[1] = K_t*aug_gain*loadcell_current[1]+HAND_GAIN*loadcell_current_hand[1]-(0.554)*sgn(EstimatedSpd[1])-1.0/120.0*EstimatedSpd[1]; // with 0.02 mu friction const
  TorqueInput[0] = K_t*aug_gain*loadcell_current[0]+HAND_GAIN*loadcell_current_hand[0]-(0.554)*sgn(EstimatedSpd[0])-1.0/120.0*EstimatedSpd[0];

  EstimatedAcc[1] = ((M_BODY*DIST_WHEELS*DIST_WHEELS*RADIUS_WHEEL*RADIUS_WHEEL + 4*I_WHEEL*DIST_WHEELS*DIST_WHEELS + I_BODY*RADIUS_WHEEL*RADIUS_WHEEL)*(2*DIST_WHEELS*TorqueInput[1] - D_MASSCENTER*M_BODY*RADIUS_WHEEL*RADIUS_WHEEL*gz/180.0*PI*_rpm[0]/60.0*2*PI + 2*D_MASSCENTER*M_WHEEL*RADIUS_WHEEL*RADIUS_WHEEL*gz/180.0*PI*_rpm[0]/60.0*2*PI))/(2*DIST_WHEELS*(2*I_WHEEL*DIST_WHEELS*DIST_WHEELS + I_BODY*RADIUS_WHEEL*RADIUS_WHEEL)*(M_BODY*RADIUS_WHEEL*RADIUS_WHEEL + 2*I_WHEEL)) + (RADIUS_WHEEL*RADIUS_WHEEL*(- M_BODY*DIST_WHEELS*DIST_WHEELS + I_BODY)*(2*DIST_WHEELS*TorqueInput[0] + D_MASSCENTER*M_BODY*RADIUS_WHEEL*RADIUS_WHEEL*gz/180.0*PI*_rpm[1]/60.0*2*PI - 2*D_MASSCENTER*M_WHEEL*RADIUS_WHEEL*RADIUS_WHEEL*gz/180.0*PI*_rpm[1]/60.0*2*PI))/(2*DIST_WHEELS*(2*I_WHEEL*DIST_WHEELS*DIST_WHEELS + I_BODY*RADIUS_WHEEL*RADIUS_WHEEL)*(M_BODY*RADIUS_WHEEL*RADIUS_WHEEL + 2*I_WHEEL));
  EstimatedAcc[0] = ((M_BODY*DIST_WHEELS*DIST_WHEELS*RADIUS_WHEEL*RADIUS_WHEEL + 4*I_WHEEL*DIST_WHEELS*DIST_WHEELS + I_BODY*RADIUS_WHEEL*RADIUS_WHEEL)*(2*DIST_WHEELS*TorqueInput[0] + D_MASSCENTER*M_BODY*RADIUS_WHEEL*RADIUS_WHEEL*gz/180.0*PI*_rpm[1]/60.0*2*PI - 2*D_MASSCENTER*M_WHEEL*RADIUS_WHEEL*RADIUS_WHEEL*gz/180.0*PI*_rpm[1]/60.0*2*PI))/(2*DIST_WHEELS*(2*I_WHEEL*DIST_WHEELS*DIST_WHEELS + I_BODY*RADIUS_WHEEL*RADIUS_WHEEL)*(M_BODY*RADIUS_WHEEL*RADIUS_WHEEL + 2*I_WHEEL)) + (RADIUS_WHEEL*RADIUS_WHEEL*(- M_BODY*DIST_WHEELS*DIST_WHEELS + I_BODY)*(2*DIST_WHEELS*TorqueInput[1] - D_MASSCENTER*M_BODY*RADIUS_WHEEL*RADIUS_WHEEL*gz/180.0*PI*_rpm[0]/60.0*2*PI + 2*D_MASSCENTER*M_WHEEL*RADIUS_WHEEL*RADIUS_WHEEL*gz/180.0*PI*_rpm[0]/60.0*2*PI))/(2*DIST_WHEELS*(2*I_WHEEL*DIST_WHEELS*DIST_WHEELS + I_BODY*RADIUS_WHEEL*RADIUS_WHEEL)*(M_BODY*RADIUS_WHEEL*RADIUS_WHEEL + 2*I_WHEEL)); 

  EstimatedSpd[1] += EstimatedAcc[1]*0.018/2/PI*60.0; 
  EstimatedSpd[0] += EstimatedAcc[0]*0.018/2/PI*60.0;   
  /*
  AvgEstacc[idx] = abs(EstimatedAcc[1]);
  AvgRealacc[idx] = abs(_accel[1]);
  AvgEstspd[idx] = abs(EstimatedSpd[1]);
  AvgRealspd[idx] = abs(_rpm[1]);
  */
  idx++;
  idx = idx % 40;
  Esum_acc += abs(EstimatedAcc[1]);
  Rsum_acc += abs(_accel[1]);
  Esum_spd += abs(EstimatedSpd[1]);
  Rsum_spd += abs(_rpm[1]);
  if(idx == 39){
    if(Esum_acc>Rsum_acc+0.1*40 && Esum_spd>Rsum_spd+7.0*40){
      M_BODY += 2;
      I_BODY += 0.12;
    }
    else if(Esum_acc+0.1*40<Rsum_acc && Esum_spd+7.0*40<Rsum_spd){
      M_BODY -= 2;
      I_BODY -= 0.12;
    }
    Esum_acc = 0; Esum_spd = 0;
    Rsum_acc = 0; Rsum_spd = 0;
    EstimatedSpd[1] = _rpm[1];
    EstimatedSpd[0] = _rpm[0];
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






















































































































































































































































































































































































































































































































































































































































































































































































































































































































































