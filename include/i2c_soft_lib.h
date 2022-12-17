#ifndef I2C_SOFT_LIB_H
#define I2C_SOFT_LIB_H

typedef bool(*read_pin_cb)(void);

typedef struct {
    read_pin_cb read_sda_ptr;
    read_pin_cb read_scl_ptr;
} i2c_soft_init;

void i2C_init(i2c_soft_init dev_ptr);

void i2C_handler(i2c_soft_init dev_ptr);


#endif