#ifndef I2C_SOFT_LIB_H
#define I2C_SOFT_LIB_H

#include <stdint.h>
#include "i2c_lib_config.h"

#ifndef DEVICE_MAX
  #warning i2c_soft_lib: DEVICE_MAX is NOT defined! DEVICE_MAX == 10
  #define DEVICE_MAX (10)
#endif

#ifndef ADDRESS_BITS_MAX
  #warning i2c_soft_lib: ADDRESS_BITS_MAX is NOT defined! ADDRESS_BITS_MAX == 7
  #define ADDRESS_BITS_MAX (7)
#endif
#ifndef ASSERT
#warning ASSERT is NOT configured!
  #define ASSERT(x) \
    if(!(x)) \
        {(void) 0}

#endif

#define BITS_IN_BYTE 8

typedef bool(*i2c_soft_read_pin_cb)(void);
typedef void(*i2c_soft_pull_pin_cb)(bool);
typedef void(*i2c_soft_delay_micros_cb)(uint32_t);

typedef enum {
  I2C_SOFT_SPEED_10K,
  I2C_SOFT_SPEED_100K,
  I2C_SOFT_SPEED_200K,
  I2C_SOFT_SPEED_400K,

  I2C_SOFT_SPEED_USE_BUS,   // only for device speed, will use current bus speed
  I2C_SOFT_SPEED_TOTAL
} i2c_soft_speed;

typedef enum {
  I2C_SOFT_DISPLAY = 0,
  I2C_SOFT_TEMP,
  I2C_SOFT_RTC,
  I2C_SOFT_BME,

  I2C_SOFT_TOTAL
} i2c_soft_devtype;

typedef enum {
  I2C_ERROR_NONE = 0,

  I2C_ERROR_TOTAL
} i2c_soft_error;

typedef struct {
  i2c_soft_read_pin_cb      read_sda_ptr;
  i2c_soft_read_pin_cb      read_scl_ptr;
  i2c_soft_pull_pin_cb      pull_sda_ptr;
  i2c_soft_pull_pin_cb      pull_scl_ptr;
  i2c_soft_delay_micros_cb  delay_micros;
  i2c_soft_speed            speed;
} i2c_soft_init_struct;

typedef struct {
  i2c_soft_read_pin_cb      read_sda_ptr;
  i2c_soft_read_pin_cb      read_scl_ptr;
  i2c_soft_pull_pin_cb      pull_sda_ptr;
  i2c_soft_pull_pin_cb      pull_scl_ptr;
  i2c_soft_delay_micros_cb  delay_micros;
  i2c_soft_speed            speed;
  uint16_t                  half_period_delay;
  uint16_t                  quarter_period_delay;
} i2c_soft_bus;

typedef struct {
  i2c_soft_devtype  type;
  uint8_t           addr;
  i2c_soft_bus*     bus_ptr;
  i2c_soft_speed    speed;
} i2c_soft_device;

void i2c_soft_init(i2c_soft_bus* bus_ptr, i2c_soft_init_struct* init_ptr);

void i2C_soft_handler(i2c_soft_bus* bus_ptr);

bool i2c_soft_read(i2c_soft_device* device_ptr, uint8_t* buff_ptr, uint8_t max_size);
bool i2c_soft_write(i2c_soft_device* device_ptr, uint8_t* buff_ptr, uint8_t size);

void i2c_soft_set_speed(i2c_soft_bus* bus_ptr, i2c_soft_speed speed);
i2c_soft_speed i2c_soft_get_speed(i2c_soft_bus* bus_ptr);


uint8_t i2c_soft_device_lookup(i2c_soft_bus* bus_ptr, i2c_soft_device* dev_arr);

#endif // I2C_SOFT_LIB_H