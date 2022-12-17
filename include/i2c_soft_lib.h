#ifndef I2C_SOFT_LIB_H
#define I2C_SOFT_LIB_H

#include <stdint.h>

#ifndef DEVICE_MAX
    #define DEVICE_MAX 10
#endif

#ifndef ASSERT
  #define ASSERT(x) \
    ((x) \
        ? (void)0 \
        : (void)0\
    )
#endif

typedef bool(*read_pin_cb)(void);

typedef struct {
  read_pin_cb read_sda_ptr;
  read_pin_cb read_scl_ptr;
} i2c_soft_init_struct;

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
} i2c_error;

typedef struct {
  i2c_soft_devtype type;
  uint8_t addr;
} i2c_soft_device;

typedef struct {
  read_pin_cb read_sda_ptr;
  read_pin_cb read_scl_ptr;
  i2c_soft_device device_arr[DEVICE_MAX];
} i2c_soft_bus;

void i2c_soft_init(i2c_soft_bus* bus_ptr, i2c_soft_init_struct* init_ptr);

void i2C_soft_handler(i2c_soft_bus* bus_ptr);

bool i2c_soft_read(i2c_soft_device* device_ptr, uint8_t* buff_ptr, uint8_t max_size);
bool i2c_soft_write(i2c_soft_device* device_ptr, uint8_t* buff_ptr, uint8_t size);

#endif // I2C_SOFT_LIB_H