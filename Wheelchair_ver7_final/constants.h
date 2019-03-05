
#ifndef CONSTANTS_H_
#define CONSTANTS_H_

#include <Arduino.h>

const int BUTTON_STATE_PRESS 	= 0;
const int BUTTON_STATE_OPEN 	= 1;

const float GAIN_BATTERY = 0.03222656; // 3.3V/1024 * 230(RESISTOR1+2) / 30(RESISTOR1)

const int POWER_STATE_CHARGING 	= 0;
const int POWER_STATE_BATTERY 	= 1;

const int LOADCELL_COUNT = 4;

const int16_t LOADCELL_RAW_DEADZONE = 30; //20
const int16_t LOADCELL_RAW_ZERO_DEADZONE = 10;
const int16_t LOADCELL_RAW_ZERO_NOISE_THRES = 200;
const int16_t LOADCELL_RAW_NOISE_THRES = 5000;

const float LOADCELL_CURRENT_GAIN_F = 0.04;   //0.020
const float LOADCELL_CURRENT_GAIN_B = 0.06;   //0.017

const float LOADCELL_CURRENT_DEADZONE = 0.5;
const float LOADCELL_CURRENT_MAX = 30;
const float VEL_MAX = 180;

const int AUTO_CALIBRATION_LIMIT =  50;

const int ONE_SEC = 1000;

const int OBSTACLE_CURRENT_LIMIT = 50;
const int OBSTACLE_CURRENT_GAP = 30;
const int BUMP_CURRENT_LIMIT = 20;
const float DNAGER_FEEDBACK = 0.3;
#define POLE_PAIR 31
#define gravity 9.81

const float K_t = 0.63; // torque constant
const int ST_COUNT = 20;
const float ST_INTENTION = 4;

const float I_WHEEL = 0.03;
const float M_WHEEL = 3.92; //mc = m_body-2m_wheel
const float DIST_WHEELS = 0.312; //2*L = 0.625
const float RADIUS_WHEEL = 0.127; //0.0825; // 0.127
const float HAND_GAIN = 0.12;

const float I_BODY_d = 3.26;
const float M_BODY_d = 42.2;
const float D_MASSCENTER_d = 0.11; 

enum cart_state_e {
  RUN,
  BRAKE_1,
  BRAKE_2,
  BRAKE_3,
  BRAKE_SLEEP,
  EMERGENCY, // Adding emergency
  SIT_DOWN, //adding SIT_DOWN
  LEFT,
  CLIMB,
  FOLDING,
};

enum side_state_e {
  NEUTRAL,
  FORWARD,
  BACKWARD,
};

enum led_state_e {
  VBAT,		 // depends on voltage
  CHG,		 // <4-3-2-1>
  CALIBRATION, // <43 - off - 21 - off>
  START_UP,	 // <4-3-2-1>
  FACTORY,	 // <A41 - A23>
  ERROR_UV,	 // <A1>
  ERROR_OV,	 // <A2>
  ERROR_CRC,	 // <A3>
};

enum loadcellPos {
  LF = 0,
  RF,
  LB,
  RB,
};

enum packetCode {
  LF_H = 0 ,
  LF_L     ,
  CRC_LF_H ,
  CRC_LF_L ,
  RF_H     ,
  RF_L     ,
  CRC_RF_H ,
  CRC_RF_L ,
  LB_H     ,
  LB_L     ,
  CRC_LB_H ,
  CRC_LB_L ,
  RB_H     ,
  RB_L     ,
  CRC_RB_H ,
  CRC_RB_L ,
  FIRST_CHECK,
};


/* CRC */
const unsigned short crc_table[256] = { 0x0000,
                                        0x8005, 0x800F, 0x000A, 0x801B, 0x001E, 0x0014, 0x8011,
                                        0x8033, 0x0036, 0x003C, 0x8039, 0x0028, 0x802D, 0x8027,
                                        0x0022, 0x8063, 0x0066, 0x006C, 0x8069, 0x0078, 0x807D,
                                        0x8077, 0x0072, 0x0050, 0x8055, 0x805F, 0x005A, 0x804B,
                                        0x004E, 0x0044, 0x8041, 0x80C3, 0x00C6, 0x00CC, 0x80C9,
                                        0x00D8, 0x80DD, 0x80D7, 0x00D2, 0x00F0, 0x80F5, 0x80FF,
                                        0x00FA, 0x80EB, 0x00EE, 0x00E4, 0x80E1, 0x00A0, 0x80A5,
                                        0x80AF, 0x00AA, 0x80BB, 0x00BE, 0x00B4, 0x80B1, 0x8093,
                                        0x0096, 0x009C, 0x8099, 0x0088, 0x808D, 0x8087, 0x0082,
                                        0x8183, 0x0186, 0x018C, 0x8189, 0x0198, 0x819D, 0x8197,
                                        0x0192, 0x01B0, 0x81B5, 0x81BF, 0x01BA, 0x81AB, 0x01AE,
                                        0x01A4, 0x81A1, 0x01E0, 0x81E5, 0x81EF, 0x01EA, 0x81FB,
                                        0x01FE, 0x01F4, 0x81F1, 0x81D3, 0x01D6, 0x01DC, 0x81D9,
                                        0x01C8, 0x81CD, 0x81C7, 0x01C2, 0x0140, 0x8145, 0x814F,
                                        0x014A, 0x815B, 0x015E, 0x0154, 0x8151, 0x8173, 0x0176,
                                        0x017C, 0x8179, 0x0168, 0x816D, 0x8167, 0x0162, 0x8123,
                                        0x0126, 0x012C, 0x8129, 0x0138, 0x813D, 0x8137, 0x0132,
                                        0x0110, 0x8115, 0x811F, 0x011A, 0x810B, 0x010E, 0x0104,
                                        0x8101, 0x8303, 0x0306, 0x030C, 0x8309, 0x0318, 0x831D,
                                        0x8317, 0x0312, 0x0330, 0x8335, 0x833F, 0x033A, 0x832B,
                                        0x032E, 0x0324, 0x8321, 0x0360, 0x8365, 0x836F, 0x036A,
                                        0x837B, 0x037E, 0x0374, 0x8371, 0x8353, 0x0356, 0x035C,
                                        0x8359, 0x0348, 0x834D, 0x8347, 0x0342, 0x03C0, 0x83C5,
                                        0x83CF, 0x03CA, 0x83DB, 0x03DE, 0x03D4, 0x83D1, 0x83F3,
                                        0x03F6, 0x03FC, 0x83F9, 0x03E8, 0x83ED, 0x83E7, 0x03E2,
                                        0x83A3, 0x03A6, 0x03AC, 0x83A9, 0x03B8, 0x83BD, 0x83B7,
                                        0x03B2, 0x0390, 0x8395, 0x839F, 0x039A, 0x838B, 0x038E,
                                        0x0384, 0x8381, 0x0280, 0x8285, 0x828F, 0x028A, 0x829B,
                                        0x029E, 0x0294, 0x8291, 0x82B3, 0x02B6, 0x02BC, 0x82B9,
                                        0x02A8, 0x82AD, 0x82A7, 0x02A2, 0x82E3, 0x02E6, 0x02EC,
                                        0x82E9, 0x02F8, 0x82FD, 0x82F7, 0x02F2, 0x02D0, 0x82D5,
                                        0x82DF, 0x02DA, 0x82CB, 0x02CE, 0x02C4, 0x82C1, 0x8243,
                                        0x0246, 0x024C, 0x8249, 0x0258, 0x825D, 0x8257, 0x0252,
                                        0x0270, 0x8275, 0x827F, 0x027A, 0x826B, 0x026E, 0x0264,
                                        0x8261, 0x0220, 0x8225, 0x822F, 0x022A, 0x823B, 0x023E,
                                        0x0234, 0x8231, 0x8213, 0x0216, 0x021C, 0x8219, 0x0208,
                                        0x820D, 0x8207, 0x0202
                                      };


#endif /* CONSTANTS_H_ */








































































































































































































































































































































































































































































































































































































































































































































































































































































































































































































