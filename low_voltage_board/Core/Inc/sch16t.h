#ifndef SCH16T_H
#define SCH16T_H

#include "common_types.h"
#include <stdbool.h>

// return false if spi fails or status reg indicates issue
bool imu_init(void);

//  returns false on spi/crc/addy error
bool imu_read(imu_data_t *out);

#endif // SCH16T_H
