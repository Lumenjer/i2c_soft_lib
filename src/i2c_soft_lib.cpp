#include "../include/i2c_soft_lib.h"

#define MAX_ADDRESS             (127)
#define I2C_ADDRESS_START       (0x08)      // firs available address I2C (0x00 - 0x07 are reserved)
#define I2C_ADDRESS_END         (0x77)      // last available address I2C (0x78 - 0x7F are reserved)
#define I2C_ADDRESS_MAX         (0x7F)      // Max support I2C device address
#define I2C_SOFT_TIMEOUT_US     (1000)      // timeout for correct line condition, range 1 - 65535
#define I2C_DELAY_1_US          (1)         // delay 1 microsecond
#define I2C_SOFT_RELEASE        (true)
#define I2C_SOFT_PULL           (false)

typedef enum {
  I2C_SOFT_WRITE  = 0u,                     ///< used for set I2C write bit
  I2C_SOFT_READ   = 1u                      ///< used for set I2C read bit
} i2c_soft_msg_type;

static bool i2c_soft_byte_read(i2c_soft_bus* bus_ptr, uint8_t* byte);
static bool i2c_soft_byte_write(i2c_soft_bus* bus_ptr, uint8_t byte);
static bool i2c_soft_transmission_start(i2c_soft_bus* bus_ptr);
static bool i2c_soft_transmission_stop(i2c_soft_bus* bus_ptr);
inline static bool i2c_soft_wait_until_line(i2c_soft_delay_micros_cb delay_micros, i2c_soft_read_pin_cb read_pin, bool condition);

///=========================================================================
/// @brief Function for init I2C bus
///=========================================================================
void i2c_soft_init
(
  i2c_soft_bus*         bus_ptr,      ///< Pointer to I2C bus
  i2c_soft_init_struct* init_ptr      ///< Pointer to I2C init struct
){
  ASSERT(bus_ptr);
  ASSERT(init_ptr);
  ASSERT(init_ptr->read_scl_ptr);
  ASSERT(init_ptr->read_sda_ptr);
  ASSERT(init_ptr->set_scl_ptr);
  ASSERT(init_ptr->set_sda_ptr);
  ASSERT(init_ptr->delay_micros);

  bus_ptr->read_scl_ptr = init_ptr->read_scl_ptr;
  bus_ptr->read_sda_ptr = init_ptr->read_sda_ptr;
  bus_ptr->set_scl_ptr = init_ptr->set_scl_ptr;
  bus_ptr->set_sda_ptr = init_ptr->set_sda_ptr;
  bus_ptr->delay_micros = init_ptr->delay_micros;

  i2c_soft_set_speed(bus_ptr, init_ptr->speed);

  bus_ptr->set_scl_ptr(I2C_SOFT_RELEASE);
  bus_ptr->set_sda_ptr(I2C_SOFT_RELEASE);

}


///=========================================================================
/// @brief Not in use for now
///=========================================================================
void i2C_soft_handler(i2c_soft_bus* bus_ptr){

}


///=========================================================================
/// @brief Function for find devices on I2C bus
/// @return Count of founded device
///=========================================================================
uint8_t i2c_soft_device_lookup
(
  i2c_soft_bus*     bus_ptr,          ///< Pointer to I2C bus
  i2c_soft_device*  dev_arr,          ///< Input array for founded device
  uint8_t           arr_size          ///< Size of device array
)
{
  ASSERT(bus_ptr);
  ASSERT(dev_arr);

  uint8_t device_founded = 0;
  i2c_soft_device temp = {
    .type     = I2C_SOFT_UNKNOWN,
    .addr     = 0,
    .bus_ptr  = bus_ptr,
    .speed    = I2C_SOFT_SPEED_USE_BUS
  };

  for (uint8_t addr = I2C_ADDRESS_START; addr <= I2C_ADDRESS_END; addr++)
  {
    temp.addr = addr;
    uint8_t data = 0;
    if (i2c_soft_read(&temp, &data, 1))
    {
      if (device_founded < arr_size)
      {
        dev_arr[device_founded] = temp;
      }
      device_founded++;
    }
  }

  return device_founded;
}

///=========================================================================
/// @brief Function for read from I2C device
/// @return result (success == true)
///=========================================================================
bool i2c_soft_read
(
  i2c_soft_device*  device_ptr,       ///< I2C device pointer
  uint8_t*          buff_ptr,         ///< Read buffer pointer
  uint8_t           max_size          ///< Max bytes to read
)
{
  ASSERT(device_ptr->addr <= I2C_ADDRESS_MAX);

  return i2c_soft_write_read(device_ptr, buff_ptr, 0, max_size);
}

///=========================================================================
/// @brief Function for write and read from I2C device
///        Can be used for writing command or address which should be read
/// @return result (success == true)
///=========================================================================
bool i2c_soft_write_read
(
  i2c_soft_device*  device_ptr,       ///< I2C device pointer
  uint8_t*          in_out_buff_ptr,  ///< Write and read buffer pointer
  uint8_t           write_size,       ///< Size to write in bytes
  uint8_t           max_read_size     ///< Max bytes to read
)
{
  ASSERT(device_ptr);
  ASSERT(device_ptr->addr <= I2C_ADDRESS_MAX);
  ASSERT(in_out_buff_ptr);

  bool result = false;
  i2c_soft_bus* bus_ptr = device_ptr->bus_ptr;
  i2c_soft_speed backup_speed = i2c_soft_get_speed(bus_ptr);

  if (device_ptr->speed != I2C_SOFT_SPEED_USE_BUS)
  {
    i2c_soft_set_speed(bus_ptr, device_ptr->speed);
  }

  result = i2c_soft_transmission_start(bus_ptr);

  if (write_size > 0)
  {
    for (uint8_t idx = 0; (idx <= write_size) && result; idx++)
    {
      if (i2c_soft_byte_write(bus_ptr, idx ? in_out_buff_ptr[idx - 1] : (uint8_t)((device_ptr->addr << 1u) + I2C_SOFT_WRITE)))
      {
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
      result = i2c_soft_transmission_start(bus_ptr);
    }
  }

  if (result)
  {
    result = i2c_soft_byte_write(bus_ptr, (uint8_t)((device_ptr->addr << 1u) + I2C_SOFT_READ));
  }
  else
  {
    // TODO:
  }

  if (result)
  {
    for (uint8_t idx = 0; idx < max_read_size; idx++)
    {
      result = i2c_soft_byte_read(bus_ptr, &in_out_buff_ptr[idx]);
      bus_ptr->set_sda_ptr(idx == (max_read_size - 1));     // send NACK if last byte
      bus_ptr->set_scl_ptr(I2C_SOFT_RELEASE);
      bus_ptr->delay_micros(bus_ptr->half_period_delay);
      i2c_soft_wait_until_line(bus_ptr->delay_micros, bus_ptr->read_scl_ptr, I2C_SOFT_RELEASE);
      bus_ptr->set_scl_ptr(I2C_SOFT_PULL);
      bus_ptr->delay_micros(bus_ptr->half_period_delay);
    }
  }
  else
  {
    // TODO:
  }

  i2c_soft_transmission_stop(bus_ptr);

  i2c_soft_set_speed(bus_ptr, backup_speed);

  return result;
}

///=========================================================================
/// @brief Function for write to I2C device
/// @return result (success == true)
///=========================================================================
bool i2c_soft_write
(
  i2c_soft_device*  device_ptr,       ///< I2C device pointer
  uint8_t*          buff_ptr,         ///< Write buffer pointer
  uint8_t           size              ///< Size to write in bytes
)
{
  ASSERT(device_ptr);
  ASSERT(device_ptr->addr <= I2C_ADDRESS_MAX);
  ASSERT(buff_ptr);

  bool result = false;
  i2c_soft_bus* bus_ptr = device_ptr->bus_ptr;
  i2c_soft_speed backup_speed = i2c_soft_get_speed(bus_ptr);

  if (device_ptr->speed != I2C_SOFT_SPEED_USE_BUS)
  {
    i2c_soft_set_speed(bus_ptr, device_ptr->speed);
  }

  result = i2c_soft_transmission_start(bus_ptr);

  for (uint8_t idx = 0; (idx <= size && result); idx++)
  {
    result = i2c_soft_byte_write(bus_ptr, idx ? buff_ptr[idx - 1] : (uint8_t)((device_ptr->addr << 1u) + I2C_SOFT_WRITE));
  }

  i2c_soft_transmission_stop(bus_ptr);

  i2c_soft_set_speed(bus_ptr, backup_speed);

  return result;
}

///=========================================================================
/// @brief Function for read byte from I2C bus
/// @return result (success == true)
///=========================================================================
static bool i2c_soft_byte_read
(
  i2c_soft_bus* bus_ptr,              ///< I2C bus pointer
  uint8_t*      byte                  ///< Byte data pointer
)
{
  bool result = true;
  *byte = 0;
  for (uint8_t bit_nums = BITS_IN_BYTE; bit_nums > 0; bit_nums--)
  {
    bus_ptr->set_scl_ptr(I2C_SOFT_RELEASE);
    bus_ptr->delay_micros(bus_ptr->half_period_delay);
    i2c_soft_wait_until_line(bus_ptr->delay_micros, bus_ptr->read_scl_ptr, I2C_SOFT_RELEASE);
    *byte |= (bus_ptr->read_sda_ptr() << bit_nums - 1);
    bus_ptr->set_scl_ptr(I2C_SOFT_PULL);
    bus_ptr->delay_micros(bus_ptr->half_period_delay);
    i2c_soft_wait_until_line(bus_ptr->delay_micros, bus_ptr->read_scl_ptr, I2C_SOFT_PULL);
  }

  return result;
}


///=========================================================================
/// @brief Function for generate I2C start or restart pattern
/// @return result (success == true)
///=========================================================================
static bool i2c_soft_transmission_start
(
  i2c_soft_bus* bus_ptr               ///< I2C bus pointer
)
{
  bool result = false;

  bus_ptr->set_sda_ptr(I2C_SOFT_RELEASE);
  bus_ptr->set_scl_ptr(I2C_SOFT_RELEASE);
  bus_ptr->delay_micros(bus_ptr->half_period_delay);

  if (!i2c_soft_wait_until_line(bus_ptr->delay_micros, bus_ptr->read_scl_ptr, I2C_SOFT_RELEASE) ||
      !i2c_soft_wait_until_line(bus_ptr->delay_micros, bus_ptr->read_scl_ptr, I2C_SOFT_RELEASE))
  {
    return false;
  }
  else
  {
    result = true;
  }

  bus_ptr->set_sda_ptr(I2C_SOFT_PULL);
  bus_ptr->delay_micros(bus_ptr->half_period_delay);
  bus_ptr->set_scl_ptr(I2C_SOFT_PULL);

  return result;
}

///=========================================================================
/// @brief Function for generate I2C stop pattern
/// @return result (success == true)
///=========================================================================
static bool i2c_soft_transmission_stop
(
  i2c_soft_bus* bus_ptr               ///< I2C bus pointer
)
{
  bool result = false;

  if (bus_ptr->read_scl_ptr())
  {
    bus_ptr->set_scl_ptr(I2C_SOFT_PULL);
    i2c_soft_wait_until_line(bus_ptr->delay_micros, bus_ptr->read_scl_ptr, I2C_SOFT_PULL);
  }

  if (bus_ptr->read_sda_ptr())
  {
    bus_ptr->set_sda_ptr(I2C_SOFT_PULL);
    i2c_soft_wait_until_line(bus_ptr->delay_micros, bus_ptr->read_sda_ptr, I2C_SOFT_PULL);
  }

  bus_ptr->set_scl_ptr(I2C_SOFT_RELEASE);
  bus_ptr->delay_micros(bus_ptr->half_period_delay);
  bus_ptr->set_sda_ptr(I2C_SOFT_RELEASE);
  bus_ptr->delay_micros(bus_ptr->half_period_delay);

  return result;
}


///=========================================================================
/// @brief Function for delay until line condition
/// @return result (success == true)
///=========================================================================
inline static bool i2c_soft_wait_until_line
(
  i2c_soft_delay_micros_cb  delay_micros,       ///< Pointer for delay microsecond function
  i2c_soft_read_pin_cb      read_pin,           ///< Pointer for read pin status function
  bool                      condition           ///< Expected line condition
)
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
/// @brief Function for write byte to I2C bus
/// @return result (success == true)
///=========================================================================
static bool i2c_soft_byte_write
(
  i2c_soft_bus* bus_ptr,              ///< I2C bus pointer
  uint8_t       byte                  ///< Byte to write
)
{
  bool result = false;

  for (uint8_t bit_num = BITS_IN_BYTE; bit_num > 0; bit_num--){
    bool release = ((byte >> (bit_num - 1)) & 1);
    bus_ptr->set_sda_ptr(release);
    bus_ptr->delay_micros(bus_ptr->quarter_period_delay);
    bus_ptr->set_scl_ptr(I2C_SOFT_RELEASE);
    bus_ptr->delay_micros(bus_ptr->half_period_delay);

    i2c_soft_wait_until_line(bus_ptr->delay_micros, bus_ptr->read_scl_ptr, I2C_SOFT_RELEASE);

    bus_ptr->set_scl_ptr(I2C_SOFT_PULL);
    bus_ptr->delay_micros(bus_ptr->quarter_period_delay);
  }

  bus_ptr->set_sda_ptr(I2C_SOFT_RELEASE);
  bus_ptr->set_scl_ptr(I2C_SOFT_RELEASE);
  bus_ptr->delay_micros(bus_ptr->half_period_delay);
  result = !bus_ptr->read_sda_ptr();
  bus_ptr->set_scl_ptr(I2C_SOFT_PULL);

  return result;
}


///=========================================================================
/// @brief Function for set I2C bus speed
///=========================================================================
void i2c_soft_set_speed
(
  i2c_soft_bus* bus_ptr,              ///< I2C bus pointer
  i2c_soft_speed speed                ///< Speed to set
)
{
  ASSERT(speed < I2C_SOFT_SPEED_TOTAL);

  switch (speed)
  {
  case I2C_SOFT_SPEED_10K:
    bus_ptr->half_period_delay    = 40;
    bus_ptr->quarter_period_delay = 20;
    break;

  case I2C_SOFT_SPEED_100K:
    bus_ptr->half_period_delay    = 4;
    bus_ptr->quarter_period_delay = 2;
    break;

  case I2C_SOFT_SPEED_200K:
    bus_ptr->half_period_delay    = 2;
    bus_ptr->quarter_period_delay = 1;
    break;

  case I2C_SOFT_SPEED_400K:
    bus_ptr->half_period_delay    = 1;
    bus_ptr->quarter_period_delay = 0;
    break;

  case I2C_SOFT_SPEED_USE_BUS:
  default:
    break;
  }

  bus_ptr->speed = speed;
}


///=========================================================================
/// @brief Function for get I2C bus speed
/// @return I2C bus speed
///=========================================================================
i2c_soft_speed i2c_soft_get_speed
(
  i2c_soft_bus* bus_ptr               ///< I2C bus pointer
)
{
  return bus_ptr->speed;
}
