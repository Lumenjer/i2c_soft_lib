#include <Arduino.h>
#include "../include/i2c_soft_lib.h"

#define MAX_ADDRESS (127)

enum {
  I2C_SOFT_READ   = 1u,
  I2C_SOFT_WRITE  = 0u
};

static uint8_t i2c_soft_byte_read(i2c_soft_bus* bus_ptr);
static bool i2c_soft_byte_write(i2c_soft_bus* bus_ptr, uint8_t byte);
static bool i2c_soft_transmission_start(i2c_soft_device* dev_ptr);

void i2c_soft_init(i2c_soft_bus* bus_ptr, i2c_soft_init_struct* init_ptr){
  ASSERT(bus_ptr);
  ASSERT(init_ptr);
  ASSERT(init_ptr->read_scl_ptr);
  ASSERT(init_ptr->read_sda_ptr);
  ASSERT(init_ptr->pull_scl_ptr);
  ASSERT(init_ptr->pull_sda_ptr);
  ASSERT(init_ptr->delay_micros);

  bus_ptr->read_scl_ptr = init_ptr->read_scl_ptr;
  bus_ptr->read_sda_ptr = init_ptr->read_sda_ptr;
  bus_ptr->pull_scl_ptr = init_ptr->pull_scl_ptr;
  bus_ptr->pull_sda_ptr = init_ptr->pull_sda_ptr;
  bus_ptr->delay_micros = init_ptr->delay_micros;

  bus_ptr->pull_scl_ptr(false);
  bus_ptr->pull_sda_ptr(false);

}

void i2C_soft_handler(i2c_soft_bus* bus_ptr){

}

uint8_t i2c_soft_device_lookup(i2c_soft_bus* bus_ptr, i2c_soft_device* dev_arr){
  uint8_t device_founded = 0;

  for (uint8_t adr = 0; adr < MAX_ADDRESS; adr++){

  }

  return device_founded;
}

bool i2c_soft_read(i2c_soft_device* device_ptr, uint8_t* buff_ptr, uint8_t max_size){
  bool result = false;
  i2c_soft_bus* bus = device_ptr->bus_ptr;
  uint32_t micros1 = micros();

  // ASSERT(i2c_soft_transmission_start(device_ptr, false));
  result = i2c_soft_transmission_start(device_ptr);

  uint8_t bytes[2] = {(uint8_t)((device_ptr->addr << 1u) + I2C_SOFT_WRITE),  0xD0};
  for (uint8_t idx = 0; (idx < sizeof(bytes)) && result; idx++) {
    if (i2c_soft_byte_write(bus, bytes[idx])) {
      result = true;
      // LOG(printf, "SDA %d\n", bus->read_sda_ptr());
    }
    else {
      result = false;
      LOG(printf, "Transmit byte failed: idx = %d\n", idx);
      break;
    }
  }

  // LOG(printf, "SDA = %d\n", bus->read_sda_ptr());
  bus->delay_micros(2);
  // bus->pull_scl_ptr(false);
  // bus->delay_micros(4);
  // bus->pull_sda_ptr(true);
  // bus->delay_micros(4);
  // bus->pull_scl_ptr(true);

  // if (result)
  // {
  //   result = i2c_soft_byte_write(bus, 0xD0);
  // }
  // else {
  //   LOG(println, "Addres send failed");
  // }

  bus->delay_micros(2);
  bus->pull_sda_ptr(true);
  bus->delay_micros(2);
  bus->pull_scl_ptr(false);
  bus->delay_micros(4);
  bus->pull_sda_ptr(false);
  bus->delay_micros(4);

  uint32_t micros2 = micros();

  LOG(printf, "Micros diff %d, result = %d\n", micros2 - micros1, result);
  LOG(printf, "SDA = %d, SCL = %d\n", bus->read_sda_ptr(), bus->read_scl_ptr());

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

static bool i2c_soft_transmission_start(i2c_soft_device* dev_ptr){
  bool result = false;
  i2c_soft_bus* bus = dev_ptr->bus_ptr;

  bus->pull_sda_ptr(false);
  bus->pull_scl_ptr(false);
  bus->delay_micros(4);

  if (bus->read_scl_ptr() == false || bus->read_sda_ptr() == false) {
    // ASSERT(0);
    LOG(printf, "SCL = %d, SDA = %d\n", bus->read_scl_ptr(), bus->read_sda_ptr());
    return false;
  }
  else {
    result = true;
  }

  bus->pull_sda_ptr(true);
  bus->delay_micros(4);
  bus->pull_scl_ptr(true);

  return result;
}

static bool i2c_soft_byte_write(i2c_soft_bus* bus_ptr, uint8_t byte){
  bool result = false;

  for (uint8_t bit_num = BITS_IN_BYTE; bit_num > 0; bit_num--){
    bool pull = !((byte >> (bit_num - 1)) & 1);
    bus_ptr->pull_sda_ptr(pull);
    bus_ptr->delay_micros(2);
    bus_ptr->pull_scl_ptr(false);
    bus_ptr->delay_micros(4);
    uint8_t timeout = 10;
    while (--timeout)
    {
      if (bus_ptr->read_scl_ptr() == true &&  // slave can hold scl until data will obtained
          bus_ptr->read_scl_ptr() != pull) {
            break;
          }
    }
    if (timeout == 0)
    {
      // TODO: Add error
    }
    bus_ptr->pull_scl_ptr(true);
    bus_ptr->delay_micros(2);
  }

  bus_ptr->pull_sda_ptr(false);
  bus_ptr->pull_scl_ptr(false);
  bus_ptr->delay_micros(3);
  result = !bus_ptr->read_sda_ptr();
  bus_ptr->pull_scl_ptr(true);

  uint8_t timeout = 100;
  while (timeout-- && !bus_ptr->read_sda_ptr()) // wait until bus will free
  {
    bus_ptr->delay_micros(1);
  }

  return result;
}