#include "util.h"
#include "constants.h"

void util_delay_sec(uint16_t sec) {
  while (sec--) {
    delay(1000);
  }
}

uint8_t util_lo_byte(int16_t w) {
  return (w & 0xff);
}

uint8_t util_hi_byte(int16_t w) {
  return ((w >> 8) & 0xff);
}

uint16_t util_update_crc(uint8_t *data_blk_ptr, uint16_t data_blk_size) {
  uint16_t crc_sum = 0;
  uint16_t table_idx = 0, blk_idx = 0;

  for (blk_idx = 0; blk_idx < data_blk_size; blk_idx++) {
    table_idx = ((uint16_t)(crc_sum >> 8) ^ *data_blk_ptr++) & 0xff;
    crc_sum = (crc_sum << 8) ^ crc_table[table_idx];
  }

  return crc_sum;
}

static int sort_asc(const void* item1, const void* item2) {
  int a = *((int*)item1);
  int b = *((int*)item2);
  return a - b;
}

void util_sort(int16_t arr[], uint16_t arr_length) {
  qsort(arr, arr_length, sizeof(int16_t), sort_asc);
}

int16_t util_average_window(int16_t arr[], int start_idx, int len) {
  int32_t sum = 0;

  for (int i = start_idx; i < (start_idx + len); i++)
    sum += arr[i];

  return (int16_t)(sum / len);
}

const float tau_vbat = 1 / (2 * PI * 0.001); //0.01 Hz
const float tau = 1 / (2 * PI * 10); //10HZ
const float ts = 0.015; //15ms

static float pre_value_vbat = 0, pre_value[4] = {0};


float util_lpf(float input, uint8_t i) {
  static float lpf_value;
  lpf_value = (tau * pre_value[i] + ts * input) / (tau + ts);
  pre_value[i] = lpf_value;
  return lpf_value;
}

float util_lpf_battery(float input) {
  static float lpf_value;
  lpf_value = (tau_vbat * pre_value_vbat + ts * input) / (tau_vbat + ts);
  pre_value_vbat = lpf_value;
  return lpf_value;
}

void util_lpf_battery_set_prevalue(float input) {
  pre_value_vbat = input;
}

uint8_t util_outlier_detect(int16_t arr[], int arrsize) {
  const float IQR_GAIN = 1.5;
  const int16_t MARGIN = 10;

  float median;
  int median_index, median_index1, median_index2;
  int Q1_index, Q1_index1, Q1_index2;
  int Q3_index, Q3_index1, Q3_index2;
  float Q1, Q3;
  float IQR;

  if (arrsize % 2 != 0) //ODD
  {
    median_index = (arrsize - 1) / 2;
    median = arr[ median_index ];
    if ( median_index % 2 == 0)
    {
      Q1_index1 = (median_index / 2) - 1;
      Q1_index2 =  Q1_index1 + 1;
      Q3_index1 = median_index + (median_index / 2);
      Q3_index2 = Q3_index1 + 1;
      Q1 = ((float)arr[Q1_index1] + (float)arr[Q1_index2]) / 2;
      Q3 = ((float)arr[Q3_index1] + (float)arr[Q3_index2]) / 2;
      IQR = (Q3 - Q1) * IQR_GAIN;
      for (int i = 0; i < Q1_index1; i++)       if ((arr[i] + MARGIN) < (Q1 - IQR)) return 1;
      for (int i = Q3_index2; i < arrsize; i++) if ((arr[i] - MARGIN) > (Q3 + IQR)) return 1;
    }
    else
    {
      Q1_index = (median_index - 1) / 2;
      Q3_index = (median_index + (median_index - 1) / 2) + 1;
      Q1 =  (float)arr[Q1_index];
      Q3 =  (float)arr[Q3_index];
      IQR = (Q3 - Q1) * IQR_GAIN;
      for (int i = 0; i < Q1_index; i++)        if ((arr[i] + MARGIN) < (Q1 - IQR))   return 1;
      for (int i = Q3_index; i < arrsize; i++)  if ((arr[i] - MARGIN) > (Q3 + IQR))   return 1;
    }
  }

  else //EVEN
  {
    median_index1 = (arrsize / 2) - 1;
    median_index2 = (arrsize / 2);
    median = ( (float)arr[ median_index1 ] + (float)arr[median_index2]) / 2;
    if (median_index2 % 2 == 0)
    {
      Q1_index1 = (median_index2 / 2) - 1;
      Q1_index2 =  Q1_index1 + 1;
      Q3_index1 = median_index2 + (median_index2 / 2) - 1;
      Q3_index2 = Q3_index1 + 1;

      Q1 = ((float)arr[Q1_index1] + (float)arr[Q1_index2]) / 2;
      Q3 = ((float)arr[Q3_index1] + (float)arr[Q3_index2])  / 2;
      IQR = (Q3 - Q1) * IQR_GAIN;

      for (int i = 0; i < Q1_index1; i++)       if ((arr[i] + MARGIN) < (Q1 - IQR)) return 1;
      for (int i = Q3_index2; i < arrsize; i++) if ((arr[i] - MARGIN) > (Q3 + IQR)) return 1;
    }

    else
    {
      Q1_index = median_index1 / 2;
      Q3_index = median_index2  +  Q1_index;
      Q1 = arr[Q1_index];
      Q3 = arr[Q3_index];
      IQR = (Q3 - Q1) * IQR_GAIN;
      for (int i = 0; i < Q1_index; i++)        if ((arr[i] + MARGIN) < (Q1 - IQR)) return 1;
      for (int i = Q3_index; i < arrsize; i++)  if ((arr[i] - MARGIN) > (Q3 + IQR)) return 1;
    }
  }

  return 0;
}

























































































































































































































































