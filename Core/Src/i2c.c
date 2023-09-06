  /* USER CODE BEGIN Header */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "i2c.h"
#include "stm32g0xx_hal.h"
//#include "cmsis_os2.h"
#include <stdbool.h>

#define I2C_WRITE 0
#define I2C_READ 1
#define BQ34_ADDR 0xAA

//extern power_running_state_enum power_mos_state;

void I2C_Delay(void)
{
	//HAL_Delay(1);
  uint64_t i, j, k;
  for (i = 0; i < 500; i++);
}

#include <stdbool.h>

#define I2C_GPIO_PORT GPIOA
#define Pin_SCL GPIO_PIN_11
#define Pin_SDA GPIO_PIN_12

#define Pin_SCL_L I2C_GPIO_PORT->ODR &= ~Pin_SCL
#define Pin_SCL_H I2C_GPIO_PORT->ODR |= Pin_SCL

#define Pin_SDA_L I2C_GPIO_PORT->ODR &= ~Pin_SDA
#define Pin_SDA_H I2C_GPIO_PORT->ODR |= Pin_SDA

#define Read_SDA_Pin HAL_GPIO_ReadPin(I2C_GPIO_PORT, Pin_SDA)

void I2C_Soft_Init(void);
bool I2C_Start(void);
bool I2C_Stop(void);
void I2C_Send_Byte(uint8_t txd);
uint8_t I2C_Read_Byte(void);
uint8_t I2C_Wait_Ack(void);
void I2C_Ack(void);
void I2C_NAck(void);

bool I2C_Write_REG(uint8_t SlaveAddress, uint8_t REG_Address, uint8_t REG_data);
uint8_t I2C_Read_REG(uint8_t SlaveAddress, uint8_t REG_Address);
bool I2C_Write_NByte(uint8_t SlaveAddress, uint8_t REG_Address, uint8_t *buf, uint8_t len);
bool I2C_Read_NByte(uint8_t SlaveAddress, uint8_t REG_Address, uint8_t *buf, uint8_t len);
bool I2C_CheckDevice(uint8_t SlaveAddress);

void SCL_Test(void)
{
	I2C_GPIO_PORT->ODR &= ~Pin_SCL;
	I2C_GPIO_PORT->ODR &= ~Pin_SDA;
	HAL_Delay(100);
	I2C_GPIO_PORT->ODR |= Pin_SCL;
	I2C_GPIO_PORT->ODR |= Pin_SDA;
	HAL_Delay(100);
	I2C_GPIO_PORT->ODR &= ~Pin_SCL;
	I2C_GPIO_PORT->ODR &= ~Pin_SDA;
}

void I2C_Soft_Init(void)
{
//  LL_GPIO_InitTypeDef GPIO_InitStructure; // ??GPIO???

//  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOB);

//  GPIO_InitStructure.Mode = LL_GPIO_MODE_OUTPUT;
//  GPIO_InitStructure.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
//  GPIO_InitStructure.Pin = Pin_SCL | Pin_SDA;
//  GPIO_InitStructure.Pull = LL_GPIO_PULL_UP;
//  GPIO_InitStructure.Speed = LL_GPIO_SPEED_LOW;
//  LL_GPIO_Init(I2C_GPIO_PORT, &GPIO_InitStructure);

  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pins : PA7 PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	
  I2C_Stop();
}

bool I2C_Start(void)
{
  Pin_SCL_H;
  Pin_SDA_H;
  I2C_Delay();
  if (!Read_SDA_Pin)
    return false;
  Pin_SDA_L;
  I2C_Delay();
  Pin_SDA_L;
  I2C_Delay();
  return true;
}

bool I2C_Stop(void)
{
	Pin_SDA_L;
  Pin_SCL_H;
  I2C_Delay();
  if (Read_SDA_Pin)
    return false;
  Pin_SDA_H;
  I2C_Delay();
  if (!Read_SDA_Pin)
    return false;
  Pin_SDA_H;
  I2C_Delay();
  return true;
}

void I2C_Ack(void)
{
  Pin_SCL_L;
  I2C_Delay();
  Pin_SDA_L;
  Pin_SCL_H;
  I2C_Delay();
  Pin_SCL_L;
  Pin_SDA_H;
  I2C_Delay();
}

void I2C_NAck(void)
{
  Pin_SCL_L;
  I2C_Delay();
  Pin_SDA_H;
  Pin_SCL_H;
  I2C_Delay();
  Pin_SCL_L;
  I2C_Delay();
}

uint8_t I2C_Wait_Ack(void)
{
  Pin_SCL_L;
  I2C_Delay();
  Pin_SDA_H;
  Pin_SCL_H;
  I2C_Delay();
  if (Read_SDA_Pin)
  {
    Pin_SCL_L;
    I2C_Delay();
    return false;
  }
  Pin_SCL_L;
  I2C_Delay();
  return true;
}

void I2C_Send_Byte(uint8_t txd)
{
  for (uint8_t i = 0; i < 8; i++)
  {
    Pin_SCL_L;
    I2C_Delay();
    if (txd & 0x80)
      Pin_SDA_H;
    else
      Pin_SDA_L;
    txd <<= 1;
    Pin_SCL_H;
    I2C_Delay();
  }
}

uint8_t I2C_Read_Byte(void)
{
  uint8_t rxd = 0;
  for (uint8_t i = 0; i < 8; i++)
  {
    rxd <<= 1;
    Pin_SCL_L;
    I2C_Delay();
    Pin_SCL_H;
    I2C_Delay();
    if (Read_SDA_Pin)
    {
      rxd |= 0x01;
    }
  }
  return rxd;
}

bool I2C_Write_REG(uint8_t SlaveAddress, uint8_t REG_Address, uint8_t REG_data)
{
  if (!I2C_Start())
    return false;
  I2C_Send_Byte(SlaveAddress);
  if (!I2C_Wait_Ack())
  {
    I2C_Stop();
    return false;
  }
  I2C_Send_Byte(REG_Address);
  if (!I2C_Wait_Ack())
  {
    I2C_Stop();
    return false;
  }
  I2C_Send_Byte(REG_data);
  if (!I2C_Wait_Ack())
  {
    I2C_Stop();
    return false;
  }
  if (!I2C_Stop())
    return false;
  return true;
}

uint8_t I2C_Read_REG(uint8_t SlaveAddress, uint8_t REG_Address)
{
  uint8_t data;
  if (!I2C_Start())
    return false;
  I2C_Send_Byte(SlaveAddress);
  if (!I2C_Wait_Ack())
  {
    I2C_Stop();
    return false;
  }
  I2C_Send_Byte(REG_Address);
  if (!I2C_Wait_Ack())
  {
    I2C_Stop();
    return false;
  }
  if (!I2C_Start())
    return false;
  I2C_Send_Byte(SlaveAddress + 1);
  if (!I2C_Wait_Ack())
  {
    I2C_Stop();
    return false;
  }
  data = I2C_Read_Byte();
  I2C_NAck();
  if (!I2C_Stop())
    return false;
  return data;
}

bool I2C_Write_NByte(uint8_t SlaveAddress, uint8_t REG_Address, uint8_t *buf, uint8_t len)
{
  if (!I2C_Start())
    return false;
  I2C_Send_Byte(SlaveAddress);
  if (!I2C_Wait_Ack())
  {
    I2C_Stop();
    return false;
  }
  I2C_Send_Byte(REG_Address);
  if (!I2C_Wait_Ack())
  {
    I2C_Stop();
    return false;
  }
  for (uint16_t i = 0; i < len; i++)
  {
    I2C_Send_Byte(buf[i]);
    if (i < len - 1)
    {
      if (!I2C_Wait_Ack())
      {
        I2C_Stop();
        return false;
      }
    }
  }
  I2C_Stop();
  return true;
}

bool I2C_Read_NByte(uint8_t SlaveAddress, uint8_t REG_Address, uint8_t *buf, uint8_t len)
{
  if (!I2C_Start())
    return false;
  I2C_Send_Byte(SlaveAddress);
  if (!I2C_Wait_Ack())
  {
    I2C_Stop();
    return false;
  }
  I2C_Send_Byte(REG_Address);
  if (!I2C_Wait_Ack())
  {
    I2C_Stop();
    return false;
  }
  if (!I2C_Start())
    return false;
  I2C_Send_Byte(SlaveAddress | 1);
  if (!I2C_Wait_Ack())
  {
    I2C_Stop();
    return false;
  }
  for (uint16_t i = 0; i < len; i++)
  {
    buf[i] = I2C_Read_Byte();
    if (i < len - 1)
    {
      I2C_Ack();
    }
  }
  I2C_NAck();
  I2C_Stop();
  return true;
}

bool I2C_CheckDevice(uint8_t SlaveAddress)
{
  if (!I2C_Start())
    return false;
  I2C_Send_Byte(SlaveAddress);
  if (!I2C_Wait_Ack())
  {
    I2C_Stop();
    return false;
  }
  if (!I2C_Stop())
    return false;
  return true;
}

void bq34z100_cmdWrite(uint8_t cmd, uint8_t data)
{
  uint8_t tx[1];

  tx[0] = data;
  I2C_Write_NByte(BQ34_ADDR, cmd, tx, 1);
}

void bq34z100_IT_enable(void)
{
  bq34z100_cmdWrite(0x21, 0);
}

uint8_t bq34z100_get_soc(void)
{
  uint8_t soc = 0;
  uint8_t cmd = 0x02;
	I2C_Read_NByte(BQ34_ADDR, cmd, &soc, 1);
  return soc;
}

uint16_t bq34z100_get_remaining_capacity(void)
{
  uint8_t cmd = 0x04;
  uint16_t remaining_capacity = 0;
	I2C_Read_NByte(BQ34_ADDR, cmd, (uint8_t*)&remaining_capacity, 2);
  return remaining_capacity;
}

uint16_t bq34z100_get_full_charge_capacity(void)
{
  uint8_t cmd = 0x06;
  uint16_t full_charge_capacity = 0;
	I2C_Read_NByte(BQ34_ADDR, cmd, (uint8_t*)&full_charge_capacity, 2);
  return full_charge_capacity;
}

uint16_t bq34z100_get_voltage(void)
{
  uint8_t cmd = 0x08;
  uint16_t voltage = 0;
	I2C_Read_NByte(BQ34_ADDR, cmd, (uint8_t*)&voltage, 2);
  return voltage;
}

int16_t bq34z100_get_average_current(void)
{
  uint8_t cmd = 0x0A;
  int16_t current = 0;
	I2C_Read_NByte(BQ34_ADDR, cmd, (uint8_t*)&current, 2);
  return current;
}

int16_t bq34z100_get_temperature(void)
{
  uint8_t cmd = 0x0C;
  int16_t temperature = 0;
	I2C_Read_NByte(BQ34_ADDR, cmd, (uint8_t*)&temperature, 2);
  return temperature;
}

uint8_t bq34z100_get_soh(void)
{
  uint8_t cmd = 0x2E;
  uint8_t soh = 0;
	I2C_Read_NByte(BQ34_ADDR, cmd, &soh, 1);
  return soh;
}

uint16_t bq34z100_get_time_to_empty(void)
{
  uint8_t cmd = 0x18;
  uint16_t time_to_empty = 0;
	I2C_Read_NByte(BQ34_ADDR, cmd, (uint8_t*)&time_to_empty, 2);
  return time_to_empty;
}

uint16_t bq34z100_get_time_to_full(void)
{
  uint8_t cmd = 0x1A;
  uint16_t time_to_full = 0;
	I2C_Read_NByte(BQ34_ADDR, cmd, (uint8_t*)&time_to_full, 2);
  return time_to_full;
}

int16_t bq34z100_get_current(void)
{
  uint8_t cmd = 0x10;
  int16_t current = 0;
	I2C_Read_NByte(BQ34_ADDR, cmd, (uint8_t*)&current, 2);
  return current;
}

uint16_t bq34z100_get_cycle_count(void)
{
  uint8_t cmd = 0x2C;
  uint16_t cycle_count = 0;
	I2C_Read_NByte(BQ34_ADDR, cmd, (uint8_t*)&cycle_count, 2);
  return cycle_count;
}

uint16_t bq34z100_get_recommended_charge_volt(void)
{
  uint8_t cmd = 0x30;
  uint16_t charge_volt = 0;
	I2C_Read_NByte(BQ34_ADDR, cmd, (uint8_t*)&charge_volt, 2);
  return charge_volt;
}

int16_t bq34z100_get_recommended_charge_current(void)
{
  uint8_t cmd = 0x32;
  int16_t charge_current = 0;
	I2C_Read_NByte(BQ34_ADDR, cmd, (uint8_t*)&charge_current, 2);
  return charge_current;
}
uint16_t bq34z100_get_flags(void)
{
  uint8_t cmd = 0x0E;
  uint16_t flags = 0;
	I2C_Read_NByte(BQ34_ADDR, cmd, (uint8_t*)&flags, 2);
  return flags;
}

void bq34z100_get_all_info(bq34_info_struct *info)
{
  info->average_current = bq34z100_get_average_current();
  HAL_Delay(1);
  info->cycle_count = bq34z100_get_cycle_count();
  HAL_Delay(1);
  info->flags.flag_bits = bq34z100_get_flags();
  HAL_Delay(1);
  info->full_charge_capacity = bq34z100_get_full_charge_capacity();
  HAL_Delay(1);
  info->recommended_charge_current = bq34z100_get_recommended_charge_current();
  HAL_Delay(1);
  info->remaing_capacity = bq34z100_get_remaining_capacity();
  HAL_Delay(1);
  info->soc = bq34z100_get_soc();
  HAL_Delay(1);
  info->soh = bq34z100_get_soh();
  HAL_Delay(1);
  info->temperature = bq34z100_get_temperature();
  HAL_Delay(1);
  info->time_to_empty = bq34z100_get_time_to_empty();
  HAL_Delay(1);
  info->time_to_full = bq34z100_get_time_to_full();
  HAL_Delay(1);
  info->voltage = bq34z100_get_voltage();
  HAL_Delay(1);
}

//void bq34z100_get_packed_batt_info(smart_batt_info_struct *info)
//{
//  info->head = 0x55;
//  info->tail = 0xAA;
//  info->batt_state = power_mos_state;
//  info->cmd_type = BATT_INFO_CMD;
//  info->data_len = (sizeof(smart_batt_info_struct) / sizeof(uint8_t));
//  info->average_current = bq34z100_get_average_current();
//  osDelay(1);
//  info->cycle_count = bq34z100_get_cycle_count();
//  osDelay(1);
//  info->full_charge_capacity = bq34z100_get_full_charge_capacity();
//  ;
//  osDelay(1);
//  info->remaing_capacity = bq34z100_get_remaining_capacity();
//  osDelay(1);
//  info->soc = bq34z100_get_soc();
//  osDelay(1);
//  info->soh = bq34z100_get_soh();
//  osDelay(1);
//  info->temperature = bq34z100_get_temperature();
//  osDelay(1);
//  info->time_to_empty = bq34z100_get_time_to_empty();
//  osDelay(1);
//  info->time_to_full = bq34z100_get_time_to_full();
//  osDelay(1);
//  info->voltage = bq34z100_get_voltage();
//}