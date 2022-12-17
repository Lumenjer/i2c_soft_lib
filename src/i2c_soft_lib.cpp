#include <Arduino.h>
#include "../include/i2c_soft_lib.h"

static uint8_t i2c_soft_byte_read(i2c_soft_bus* bus_ptr);
static uint8_t i2c_soft_byte_write(i2c_soft_bus* bus_ptr);

void i2c_soft_init(i2c_soft_bus* bus_ptr, i2c_soft_init_struct* init_ptr){
  ASSERT(bus_ptr);
  ASSERT(init_ptr);

  bus_ptr->read_scl_ptr = init_ptr->read_scl_ptr;
  bus_ptr->read_sda_ptr = init_ptr->read_sda_ptr;

}

void i2C_soft_handler(i2c_soft_bus* bus_ptr){

}

bool i2c_soft_read(i2c_soft_device* device_ptr, uint8_t* buff_ptr, uint8_t max_size){
  bool result = false;

  return result;
}

bool i2c_soft_write(i2c_soft_device* device_ptr, uint8_t* buff_ptr, uint8_t size){
  bool result = false;

  return result;
}

static uint8_t i2c_soft_byte_read(i2c_soft_bus* bus_ptr){
  uint8_t byte = 0;

  return byte;
}

static uint8_t i2c_soft_byte_write(i2c_soft_bus* bus_ptr){
  uint8_t byte = 0;

  return byte;
}