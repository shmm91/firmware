/*
 * Copyright (c) 2017, James Jackson and Daniel Koch, BYU MAGICC Lab
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "airbourne_board.h"

namespace rosflight_firmware
{

AirbourneBoard::AirbourneBoard()
{
}

void AirbourneBoard::init_board()
{
  systemInit();
  led2_.init(LED2_GPIO, LED2_PIN);
  led1_.init(LED1_GPIO, LED1_PIN);

  int_i2c_.init(&i2c_config[BARO_I2C]);
  ext_i2c_.init(&i2c_config[EXTERNAL_I2C]);
  spi1_.init(&spi_config[MPU6000_SPI]);
  spi3_.init(&spi_config[FLASH_SPI]);
  vcp_.init();
  
  ins_uart_.init(&uart_config[UART1], 3000000);
  
//  sbus_uart_.init(&uart_config[UART3], 100000, UART::MODE_8E2);
//  inv_pin_.init(SBUS_INV_GPIO, SBUS_INV_PIN, GPIO::OUTPUT);
}

void AirbourneBoard::board_reset(bool bootloader)
{
  (void)bootloader;
  NVIC_SystemReset();
}

// clock
uint32_t AirbourneBoard::clock_millis()
{
  return millis();
}

uint64_t AirbourneBoard::clock_micros()
{
  return micros();
}

void AirbourneBoard::clock_delay(uint32_t milliseconds)
{
  delay(milliseconds);
}

// serial
void AirbourneBoard::serial_init(uint32_t baud_rate)
{
  (void)baud_rate;
}

void AirbourneBoard::serial_write(const uint8_t *src, size_t len)
{
  vcp_.write(src, len);
}

uint16_t AirbourneBoard::serial_bytes_available()
{
  return vcp_.rx_bytes_waiting();
}

uint8_t AirbourneBoard::serial_read()
{
  return vcp_.read_byte();
}

void AirbourneBoard::serial_flush()
{
  vcp_.flush();
}

// sensors
void AirbourneBoard::sensors_init()
{
  while(millis() < 50) {} // wait for sensors to boot up
  
  ins_.init(&ins_uart_);
  
  if (!ins_present())
  {
    imu_.init(&spi1_);
    baro_.init(&int_i2c_);
    mag_.init(&int_i2c_);
  }
  airspeed_.init(&ext_i2c_);

}

uint16_t AirbourneBoard::num_sensor_errors()
{
  return ext_i2c_.num_errors();
}

bool AirbourneBoard::new_imu_data()
{
  if (ins_present())
    return ins_.new_imu_data();
  else
    return imu_.new_data();
}

bool AirbourneBoard::imu_read(float accel[3], float* temperature, float gyro[3], uint64_t* time_us)
{
  if (ins_present())
  {
    uint32_t time_ms;
    ins_.read_IMU(gyro, accel, &time_ms);
    *time_us = time_ms * 1000;
  }
  else
  {
    float read_accel[3], read_gyro[3];
    imu_.read(read_accel, read_gyro, temperature, time_us);
  
    accel[0] = -read_accel[1];
    accel[1] = -read_accel[0];
    accel[2] = -read_accel[2];
  
    gyro[0] = -read_gyro[1];
    gyro[1] = -read_gyro[0];
    gyro[2] = -read_gyro[2];
  }

  return true;
}

void AirbourneBoard::imu_not_responding_error()
{
  sensors_init();
}

bool AirbourneBoard::mag_present()
{
  if (ins_present())
  {
    return true;
  }
  mag_.update();
  return mag_.present();
}

void AirbourneBoard::mag_update()
{
  if (ins_present())
    return;
  mag_.update();
}

void AirbourneBoard::mag_read(float mag[3])
{
  if (ins_present())
  {
    float baro;
    uint32_t time_ms;
    ins_.read_other_sensors(mag, &baro, &time_ms);
  }
  else
  {
    mag_.update();
    mag_.read(mag);
  }
}
bool AirbourneBoard::baro_present()
{
  if (ins_present())
    return true;
  else
    baro_.update();
  return baro_.present();
}

void AirbourneBoard::baro_update()
{
  if (ins_present())
    return;
  else
    baro_.update();
}

void AirbourneBoard::baro_read(float *pressure, float *temperature)
{
  if (ins_present())
  {
    float mag[3];
    uint32_t time_ms;
    ins_.read_other_sensors(mag, pressure, &time_ms);
  }
  else
  {
    baro_.update();
    baro_.read(pressure, temperature);
  }
}

bool AirbourneBoard::diff_pressure_present()
{
  airspeed_.update();
  return airspeed_.present();
}

void AirbourneBoard::diff_pressure_update()
{
  airspeed_.update();
}


void AirbourneBoard::diff_pressure_read(float *diff_pressure, float *temperature)
{
  (void) diff_pressure;
  (void) temperature;
  airspeed_.update();
  airspeed_.read(diff_pressure, temperature);
}

bool AirbourneBoard::sonar_present()
{
  return false;
}

void AirbourneBoard::sonar_update()
{
  return;
}

float AirbourneBoard::sonar_read()
{
  return 0.0;
}

bool AirbourneBoard::ins_present()
{
  return ins_.present();
}

void AirbourneBoard::ins_update()
{
  return ins_.update();
}

bool AirbourneBoard::ins_fix()
{
  return ins_.got_fix();
}

void AirbourneBoard::ins_read(float pos[3], float vel[3], float q[4], uint64_t* time_us)
{
  uint32_t time_ms;
  ins_.read_INS(pos, vel, q, &time_ms);
  *time_us = time_ms * 1000; // INS reports ms
}

void AirbourneBoard::reset_ins_origin()
{
  ins_.set_current_pos_as_refLLa();
}


// PWM
void AirbourneBoard::rc_init(rc_type_t rc_type)
{
  switch (rc_type)
  {
  case RC_TYPE_SBUS:
    rc_sbus_.init(&inv_pin_, &sbus_uart_);
    rc_ = &rc_sbus_;
    break;
  case RC_TYPE_PPM:
  default:
    rc_ppm_.init(&pwm_config[RC_PPM_PIN]);
    rc_ = &rc_ppm_;
    break;
  }
}

float AirbourneBoard::rc_read(uint8_t channel)
{
  return rc_->read(channel);
}

void AirbourneBoard::pwm_init(uint32_t refresh_rate, uint16_t idle_pwm)
{
  for (int i = 0; i < PWM_NUM_OUTPUTS; i++)
  {
    esc_out_[i].init(&pwm_config[i], refresh_rate, 2000, 1000);
    esc_out_[i].writeUs(idle_pwm);
  }
}

void AirbourneBoard::pwm_write(uint8_t channel, float value)
{
  esc_out_[channel].write(value);
}

bool AirbourneBoard::rc_lost()
{
  return rc_->lost();
}

// non-volatile memory
void AirbourneBoard::memory_init()
{
  return flash_.init(&spi3_);
}

bool AirbourneBoard::memory_read(void * data, size_t len)
{
  return flash_.read_config(reinterpret_cast<uint8_t*>(data), len);
}

bool AirbourneBoard::memory_write(const void * data, size_t len)
{
  return flash_.write_config(reinterpret_cast<const uint8_t*>(data), len);
}

// LED
void AirbourneBoard::led0_on() { led1_.on(); }
void AirbourneBoard::led0_off() { led1_.off(); }
void AirbourneBoard::led0_toggle() { led1_.toggle(); }

void AirbourneBoard::led1_on() { led2_.on(); }
void AirbourneBoard::led1_off() { led2_.off(); }
void AirbourneBoard::led1_toggle() { led2_.toggle(); }

} // namespace rosflight_firmware
