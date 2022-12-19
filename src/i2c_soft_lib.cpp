#include <Arduino.h>
#include "../include/i2c_soft_lib.h"

#define MAX_ADDRESS             (127)
#define I2C_ADDRESS_START       (0x08)      // firs available address I2C (0x00 - 0x07 are reserved)
#define I2C_ADDRESS_END         (0x77)      // last available address I2C (0x78 - 0x7F are reserved)
#define I2C_SOFT_TIMEOUT_US     (1000)      // timeout for correct line condition, range 1 - 65535
#define I2C_DELAY_1_US          (1)         // TODO:

typedef enum {
  I2C_SOFT_READ   = 1u,
  I2C_SOFT_WRITE  = 0u
} i2c_soft_msg_type;

static bool i2c_soft_byte_read(i2c_soft_bus* bus_ptr, uint8_t* byte);
static bool i2c_soft_byte_write(i2c_soft_bus* bus_ptr, uint8_t byte);
static bool i2c_soft_transmission_start(i2c_soft_device* dev_ptr);
static bool i2c_soft_transmission_stop(i2c_soft_device* dev_ptr);
inline static bool i2c_soft_wait_until_line(i2c_soft_delay_micros_cb delay_micros, i2c_soft_read_pin_cb read_pin, bool condition);

///=========================================================================
/// @brief @todo
///=========================================================================
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


///=========================================================================
/// @brief @todo
///=========================================================================
void i2C_soft_handler(i2c_soft_bus* bus_ptr){

}


///=========================================================================
/// @brief @todo
///=========================================================================
uint8_t i2c_soft_device_lookup(i2c_soft_bus* bus_ptr, i2c_soft_device* dev_arr){
  ASSERT(bus_ptr);
  ASSERT(dev_arr);

  uint8_t device_founded = 0;

  for (uint8_t addr = I2C_ADDRESS_START; addr <= I2C_ADDRESS_END; addr++){

  }

  return device_founded;
}

///=========================================================================
/// @brief @todo
///=========================================================================
bool i2c_soft_read(i2c_soft_device* device_ptr, uint8_t* buff_ptr, uint8_t max_size){
  ASSERT(device_ptr);
  ASSERT(buff_ptr);

  bool result = false;
  i2c_soft_bus* bus = device_ptr->bus_ptr;
  uint32_t micros1 = micros();

  result = i2c_soft_transmission_start(device_ptr);

  uint8_t bytes[2] = {(uint8_t)((device_ptr->addr << 1u) + I2C_SOFT_WRITE),  0xD0};
  for (uint8_t idx = 0; (idx < sizeof(bytes)) && result; idx++) {
    if (i2c_soft_byte_write(bus, bytes[idx])) {
      result = true;
    }
    else {
      result = false;
      // TODO:
      break;
    }
  }

  if (result)
  {
    result = i2c_soft_transmission_start(device_ptr);
  }

  if (result)
  {
    result = i2c_soft_byte_write(bus, (uint8_t)((device_ptr->addr << 1u) + I2C_SOFT_READ));
  }
  else {
    // TODO:
  }

  if (result)
  {
    for (uint8_t idx = 0; idx < max_size; idx++)
    {
      result = i2c_soft_byte_read(bus, &buff_ptr[idx]);
      bus->pull_sda_ptr(idx < (max_size - 1));     // send NACK if last byte
      bus->pull_scl_ptr(false);
      bus->delay_micros(4);
      i2c_soft_wait_until_line(bus->delay_micros, bus->read_scl_ptr, true);
      bus->pull_scl_ptr(true);
      bus->delay_micros(4);
    }
  }
  else {
    // TODO:
  }

  i2c_soft_transmission_stop(device_ptr);

  uint32_t micros2 = micros();

  LOG(printf, "Micros diff %d, result = %d\n", micros2 - micros1, result);
  LOG(printf, "SDA = %d, SCL = %d\n", bus->read_sda_ptr(), bus->read_scl_ptr());

  return result;
}

///=========================================================================
/// @brief @todo
///=========================================================================
bool i2c_soft_write(i2c_soft_device* device_ptr, uint8_t* buff_ptr, uint8_t size){
  bool result = false;

  return result;
}

///=========================================================================
/// @brief @todo
///=========================================================================
static bool i2c_soft_byte_read(i2c_soft_bus* bus_ptr, uint8_t* byte){
  bool result = true;
  *byte = 0;
  for (uint8_t bit_nums = BITS_IN_BYTE; bit_nums > 0; bit_nums--) {
    bus_ptr->pull_scl_ptr(false);
    bus_ptr->delay_micros(4);
    i2c_soft_wait_until_line(bus_ptr->delay_micros, bus_ptr->read_scl_ptr, true);
    *byte |= (bus_ptr->read_sda_ptr() << bit_nums - 1);
    bus_ptr->pull_scl_ptr(true);
    bus_ptr->delay_micros(4);
    i2c_soft_wait_until_line(bus_ptr->delay_micros, bus_ptr->read_scl_ptr, false);
  }

  return result;
}


///=========================================================================
/// @brief @todo
///=========================================================================
static bool i2c_soft_transmission_start(i2c_soft_device* dev_ptr){
  bool result = false;
  i2c_soft_bus* bus = dev_ptr->bus_ptr;

  bus->pull_sda_ptr(false);
  bus->pull_scl_ptr(false);
  bus->delay_micros(4);

  if (bus->read_scl_ptr() == false || bus->read_sda_ptr() == false) {
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

///=========================================================================
/// @brief @todo
///=========================================================================
static bool i2c_soft_transmission_stop(i2c_soft_device* dev_ptr)
{
  bool result = false;
  i2c_soft_bus* bus = dev_ptr->bus_ptr;

  if (bus->read_scl_ptr())
  {
    bus->pull_scl_ptr(true);
    i2c_soft_wait_until_line(bus->delay_micros, bus->read_scl_ptr, false);
  }

  if (bus->read_sda_ptr())
  {
    bus->pull_sda_ptr(true);
    i2c_soft_wait_until_line(bus->delay_micros, bus->read_sda_ptr, false);
  }

  bus->pull_scl_ptr(false);
  bus->delay_micros(4);
  bus->pull_sda_ptr(false);
  bus->delay_micros(4);

  return result;
}


///=========================================================================
/// @brief @todo
///=========================================================================
inline static bool i2c_soft_wait_until_line(i2c_soft_delay_micros_cb delay_micros, i2c_soft_read_pin_cb read_pin, bool condition)
{
  uint16_t timeout = I2C_SOFT_TIMEOUT_US;

  while (--timeout && read_pin() != condition)
  {
    delay_micros(I2C_DELAY_1_US);
  }

  if (timeout)
  {
    return true;
  }

  return false;
}

///=========================================================================
/// @brief @todo
///=========================================================================
static bool i2c_soft_byte_write(i2c_soft_bus* bus_ptr, uint8_t byte){
  bool result = false;

  for (uint8_t bit_num = BITS_IN_BYTE; bit_num > 0; bit_num--){
    bool pull = !((byte >> (bit_num - 1)) & 1);
    bus_ptr->pull_sda_ptr(pull);
    bus_ptr->delay_micros(2);
    bus_ptr->pull_scl_ptr(false);
    bus_ptr->delay_micros(4);

    i2c_soft_wait_until_line(bus_ptr->delay_micros, bus_ptr->read_scl_ptr, true);

    bus_ptr->pull_scl_ptr(true);
    bus_ptr->delay_micros(2);
  }

  bus_ptr->pull_sda_ptr(false);
  bus_ptr->pull_scl_ptr(false);
  bus_ptr->delay_micros(4);
  result = !bus_ptr->read_sda_ptr();
  bus_ptr->pull_scl_ptr(true);

  return result;
}
