/*
    Copyright (C) 2018  NAVER LABS Corp.

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <https://www.gnu.org/licenses/>.

    
    Wheelchair VER 4
    NAVER LABS, LHT, 20181121
    3

    FEATURED BY:
  
    VESCuino UART
    NAVER LABS, CDI. 20180419

*/

#include "vescuino_uart_arduino.h"
#include "vescuino_common.h"
#include "Wheelchair.h"
#include <Wire.h>

#include <malloc.h>
#include <stdlib.h>
#include <stdio.h>

extern char _end;
extern "C" char *sbrk(int i);
char *ramstart = (char *)0x20070000;
char *ramend = (char *)0x20088000;

/* =========== UART Setting =========== */
#define NUM_OF_VESC    2   // Set Number of VESC (1~3)
/* =========== End of UART Setting =========== */

// Use Hardware Serial For Vesc (Serial1, Serial2, Serial2)
VESC_UART vesc1(&Serial1, 115200);
VESC_UART vesc2(&Serial2, 115200);

// Vesc Control Class
const int num_of_wheel = NUM_OF_VESC;
const int degree_of_freedom = NUM_OF_VESC;
VESC_CONTROL vesc_ctrl(num_of_wheel, degree_of_freedom);

Wheelchair Cart(&vesc1, &vesc2, &vesc_ctrl);

VESC_UART* _vesc_1;
VESC_UART* _vesc_2;
VESC_CONTROL* _vesc_ctrl;

void setup()
{
  //pinMode(RESET_PIN, INPUT);
  //digitalWrite(RESET_PIN, HIGH);
  //pinMode(RESET_PIN, OUTPUT);
  Serial.begin(115200);
  delay(200);
  //pinMode(RESET_PIN, OUTPUT);
  //digitalWrite(RESET_PIN, LOW); 
  Wire1.begin();
  
  _vesc_1 = &vesc1;
  _vesc_2 = &vesc2;
  _vesc_ctrl = &vesc_ctrl;
  
  bldc_interface_set_rx_fw_func(bldc_fw_received);
  bldc_interface_set_rx_value_func(bldc_val_received);
  Cart.setup();
  /*
 char *heapend = sbrk(0);
  register char * stack_ptr asm ("sp");
  struct mallinfo mi = mallinfo();
  Serial.print("Dynamic ram used: ");
  Serial.println(mi.uordblks);
  Serial.print("Program static ram used");
  Serial.println(&_end - ramstart);
  Serial.print("Stack ram used: ");
  Serial.println(ramend - stack_ptr);
  Serial.print("My guess at free mem: n"); 
  Serial.println(stack_ptr - heapend + mi.fordblks);
*/
  delay(100);
}

void loop()
{
  
  loop_time_handle_uart();
  //Serial.print("x");
    
  Cart.run();

  loop_time_delay_uart();
}





//==============VESCuino Functions==============

void bldc_fw_received(int major, int minor)
{
  /* add program code here */
  /* ---------------------------- */
#ifdef USE_PRINT_RX_RESULT
  Serial.print(F("[ bldc ] COMM_FW_VERSION = "));
  Serial.print(major);
  Serial.print(F("."));
  Serial.println(minor);
#endif
}

void bldc_val_received(mc_values *val)
{
  /* add program code here */
  uint8_t id = 0;
  // uart serial number
  id = get_current_serial_number();

  // read values as vesc_ctrl's member
  vesc_ctrl.erpm[id] = val->rpm;
  //vesc_ctrl.tacho[id] = val->tachometer;
  vesc_ctrl.current[id] = val->current_motor;
  vesc_ctrl.enc_pos[id] = val->pid_pos_now;
  /* ---------------------------- */
}
//=============================================






























































































































































































































































































































































































































































































































































































































































































































































































































































































































































