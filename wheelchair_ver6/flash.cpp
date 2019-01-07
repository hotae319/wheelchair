#include "flash.h"
#include "util.h"

DueFlashStorage dueFlashStorage;

void flash_loadcell(int16_t loadcell[]) {

  uint8_t val_in[LOADCELL_COUNT][2] = {{0,}};
  uint8_t crc_in[LOADCELL_COUNT][2] = {{0,}};

  int16_t crcval;

  for (int i = 0; i < LOADCELL_COUNT; i++) {
    val_in[i][1] = util_hi_byte(loadcell[i]);
    val_in[i][0] = util_lo_byte(loadcell[i]);
    crcval = util_update_crc(val_in[i], 2);
    crc_in[i][1] = util_hi_byte(crcval);
    crc_in[i][0] = util_lo_byte(crcval);
  }

  for (int i = 0; i < LOADCELL_COUNT; i++) {
    dueFlashStorage.write(((i * LOADCELL_COUNT) + 0) ,  val_in[i][1]);
    dueFlashStorage.write(((i * LOADCELL_COUNT) + 1) ,  val_in[i][0]);
    dueFlashStorage.write(((i * LOADCELL_COUNT) + 2) ,  crc_in[i][1]);
    dueFlashStorage.write(((i * LOADCELL_COUNT) + 3) ,  crc_in[i][0]);
  }

  if (dueFlashStorage.read(FIRST_CHECK) == 0xFF)
    dueFlashStorage.write(FIRST_CHECK, 1);
}

uint8_t flash_read_u8(packetCode p) {
  return dueFlashStorage.read(p);
}

int16_t flash_read_16(packetCode p) {
  uint8_t val_H, val_L;
  int16_t val;

  val_H = dueFlashStorage.read(p);
  val_L = dueFlashStorage.read(p + 1);
  val = ( val_L << 0 | val_H << 8);
  return val;
}










































































































































































































































































