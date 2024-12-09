#include <stdint.h>

/* parsing sbus packet to 11bit channels
 * param:
 * buff: 25 bytes message from sbus
 * channel: parsed channels */
void parse_buffer(uint8_t buff[25], uint16_t channel[16]) {

  // masking byte shiftings bits (value in hexa '0x07FF')
  uint16_t mask = 0x7ff;

  // creating channels
  channel[0] = ((buff[1] | buff[2] << 8) & mask);
  channel[1] = ((buff[2] >> 3 | buff[3] << 5) & mask);
  channel[2] = ((buff[3] >> 6 | buff[4] << 2 | buff[5] << 10) & mask);
  channel[3] = ((buff[5] >> 1 | buff[6] << 7) & mask);
  channel[4] = ((buff[6] >> 4 | buff[7] << 4) & mask);
  channel[5] = ((buff[7] >> 7 | buff[8] << 1 | buff[9] << 9) & mask);
  channel[6] = ((buff[9] >> 2 | buff[10] << 6) & mask);
  channel[7] = ((buff[10] >> 5 | buff[11] << 3) & mask);
  channel[8] = ((buff[12] | buff[13] << 8) & mask);
  channel[9] = ((buff[13] >> 3 | buff[14] << 5) & mask);
  channel[10] = ((buff[14] >> 6 | buff[15] << 2 | buff[16] << 10) & mask);
  channel[11] = ((buff[16] >> 1 | buff[17] << 7) & mask);
  channel[12] = ((buff[17] >> 4 | buff[18] << 4) & mask);
  channel[13] = ((buff[18] >> 7 | buff[19] << 1 | buff[20] << 9) & mask);
  channel[14] = ((buff[20] >> 2 | buff[21] << 6) & mask);
  channel[15] = ((buff[21] >> 5 | buff[22] << 3) & mask);
}
/*parity checking for sbus packets
 * param:
 * parity_byte: checking sbus parity
 * returns 0 if packet is correct*/
int parity_checker(int parity_byte) {
  uint8_t frame_lost, fail_safe;

  // extracting first two bit of parity byte
  frame_lost = parity_byte >> 7;
  fail_safe = (parity_byte >> 6) << 1;

  if (frame_lost == 0 && fail_safe == 0)
    return 0;

  else
    return 1;
}
