#ifndef MMC5983_H
#define MMC5983_H

#include "common_types.h"
#include <stdbool.h>

bool mag_init(void);
bool mag_read(mag_data_t *out);

#endif // MMC5983_H