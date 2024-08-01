#include "rsapriltag/tagCustom20h10.h"
#include <stdlib.h>

static uint64_t codedata[4] = {
    0x0000000000048ac7UL,
    0x00000000000ec81aUL,
    0x0000000000063171UL,
    0x000000000005f23fUL,
};
apriltag_family_t *tagCustom20h10_create() {
  apriltag_family_t *tf = calloc(1, sizeof(apriltag_family_t));
  tf->name = strdup("tagCustom20h10");
  tf->h = 10;
  tf->ncodes = 4;
  tf->codes = codedata;
  tf->nbits = 20;
  tf->bit_x = calloc(20, sizeof(uint32_t));
  tf->bit_y = calloc(20, sizeof(uint32_t));
  tf->bit_x[0] = 1;
  tf->bit_y[0] = -2;
  tf->bit_x[1] = 2;
  tf->bit_y[1] = -2;
  tf->bit_x[2] = 3;
  tf->bit_y[2] = -2;
  tf->bit_x[3] = 4;
  tf->bit_y[3] = -2;
  tf->bit_x[4] = 5;
  tf->bit_y[4] = -2;
  tf->bit_x[5] = 8;
  tf->bit_y[5] = 1;
  tf->bit_x[6] = 8;
  tf->bit_y[6] = 2;
  tf->bit_x[7] = 8;
  tf->bit_y[7] = 3;
  tf->bit_x[8] = 8;
  tf->bit_y[8] = 4;
  tf->bit_x[9] = 8;
  tf->bit_y[9] = 5;
  tf->bit_x[10] = 5;
  tf->bit_y[10] = 8;
  tf->bit_x[11] = 4;
  tf->bit_y[11] = 8;
  tf->bit_x[12] = 3;
  tf->bit_y[12] = 8;
  tf->bit_x[13] = 2;
  tf->bit_y[13] = 8;
  tf->bit_x[14] = 1;
  tf->bit_y[14] = 8;
  tf->bit_x[15] = -2;
  tf->bit_y[15] = 5;
  tf->bit_x[16] = -2;
  tf->bit_y[16] = 4;
  tf->bit_x[17] = -2;
  tf->bit_y[17] = 3;
  tf->bit_x[18] = -2;
  tf->bit_y[18] = 2;
  tf->bit_x[19] = -2;
  tf->bit_y[19] = 1;
  tf->width_at_border = 7;
  tf->total_width = 11;
  tf->reversed_border = true;
  return tf;
}

void tagCustom20h10_destroy(apriltag_family_t *tf) {
  free(tf->bit_x);
  free(tf->bit_y);
  free(tf->name);
  free(tf);
}
