/*
 * BMx280
 * BME280/BMP280 Bosch pressure sensor library
 * by Vadym Zakovinko
 * http://github.com/quard/bmx280/
 * 
 * 
 * MIT License
 * Copyright (c) 2020 Vadym Zakovinko
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#ifndef INC_BMX280_H_
#define INC_BMX280_H_

#include "math.h"
#include "stm32l0xx_ll_gpio.h"
#include "stm32l0xx_ll_i2c.h"
#include "stm32l0xx_ll_utils.h"

#ifdef __cplusplus
extern "C" {
#endif

#define BMx280_OK           0
#define BMx280_ERROR        1
#define BMx280_I2C_ERROR    2
#define BMx280_UNKNOWN_CHIP 99

#define BMx280_REG_ID         0xD0
#define BMx280_REG_SOFTRESET  0xE0
#define BMx280_REG_CONFIG     0xF5
#define BMx280_REG_CTRL_HUM   0xF2
#define BMx280_REG_CTRL_MEAS  0xF4
#define BMx280_REG_STATUS     0xF3
#define BMx280_REG_PRESSUREDATA  0xF7
#define BMx280_REG_TEMPDATA      0xFA
#define BMx280_REG_HUMIDDATA     0xFD
#define BMx280_REG_DIG_T1     0x88
#define BMx280_REG_DIG_T2     0x8A
#define BMx280_REG_DIG_T3     0x8C
#define BMx280_REG_DIG_P1     0x8E
#define BMx280_REG_DIG_P2     0x90
#define BMx280_REG_DIG_P3     0x92
#define BMx280_REG_DIG_P4     0x94
#define BMx280_REG_DIG_P5     0x96
#define BMx280_REG_DIG_P6     0x98
#define BMx280_REG_DIG_P7     0x9A
#define BMx280_REG_DIG_P8     0x9C
#define BMx280_REG_DIG_P9     0x9E
#define BMx280_REG_DIG_H1     0xA1
#define BMx280_REG_DIG_H2     0xE1
#define BMx280_REG_DIG_H3     0xE3
#define BMx280_REG_DIG_H4     0xE4
#define BMx280_REG_DIG_H5     0xE5
#define BMx280_REG_DIG_H6     0xE7

#define BMx280_SOFTRESET_VALUE  0xB6

#define BMx280_ID_BME280  0x60
#define BMx280_ID_BMP280  0x58

typedef struct {
  uint8_t im_update : 1;
  uint8_t reserved  : 2;
  uint8_t measuring : 1;
} BMx280_Status;

typedef enum {
  BMx280_TIMESTBY_0_5  = 0b000,
  BMx280_TIMESTBY_62_5 = 0b001,
  BMx280_TIMESTBY_125  = 0b010,
  BMx280_TIMESTBY_250  = 0b011,
  BMx280_TIMESTBY_500  = 0b100,
  BMx280_TIMESTBY_1000 = 0b101,
  BMx280_TIMESTBY_10   = 0b110,
  BMx280_TIMESTBY_20   = 0b111,
} BMx280_T_SB;

typedef enum {
  BMx280_FILTER_OFF = 0b000,
  BMx280_FILTER_2   = 0b001,
  BMx280_FILTER_4   = 0b010,
  BMx280_FILTER_8   = 0b011,
  BMx280_FILTER_16  = 0b100,
} BMx280_FILTER;

typedef struct {
  uint8_t spi3w_en     : 1;
  uint8_t unused       : 1;
  BMx280_FILTER filter : 3;
  BMx280_T_SB t_sb     : 3;   // time of standby
} BMx280_Config;

typedef enum {
  BMx280_OVERSAMPLING_OFF = 0b000,
  BMx280_OVERSAMPLING_1   = 0b001,
  BMx280_OVERSAMPLING_2   = 0b010,
  BMx280_OVERSAMPLING_4   = 0b011,
  BMx280_OVERSAMPLING_8   = 0b100,
  BMx280_OVERSAMPLING_16  = 0b101,
} BMx280_OVERSAMPLING;

typedef struct {
  BMx280_OVERSAMPLING osrs_t;
  BMx280_OVERSAMPLING osrs_p;
  BMx280_OVERSAMPLING osrs_h;
} BMx280_Oversampling;

typedef enum {
  BMx280_MODE_SLEEP  = 0b00,
  BMx280_MODE_FORCED = 0b01,
  BMx280_MODE_NORMAL = 0b11,
} BMx280_MODE;

typedef struct {
  uint16_t dig_T1;
  int16_t  dig_T2;
  int16_t  dig_T3;
  uint16_t dig_P1;
  int16_t  dig_P2;
  int16_t  dig_P3;
  int16_t  dig_P4;
  int16_t  dig_P5;
  int16_t  dig_P6;
  int16_t  dig_P7;
  int16_t  dig_P8;
  int16_t  dig_P9;
  uint8_t  dig_H1;
  int16_t  dig_H2;
  uint8_t  dig_H3;
  int16_t  dig_H4;
  int16_t  dig_H5;
  int8_t   dig_H6;
  } BMx280_CalibData;

typedef struct {
  I2C_TypeDef      *hi2c;
  uint8_t           address;   // 0x76 or 0x77
  uint8_t           chip_id;
  BMx280_CalibData  calib_data;
  int32_t           _fine_temperature;
  int32_t           adc_temperature;
  int32_t           adc_pressure;
  int32_t           adc_humidity;
} BMx280_HandleTypeDef;


uint8_t BMx280_Init(BMx280_HandleTypeDef *bmx280);
uint8_t BMx280_GetStatus(BMx280_HandleTypeDef *bmx280, BMx280_Status *status);
uint8_t BMx280_GetConfig(BMx280_HandleTypeDef *bmx280, BMx280_Config *config);
uint8_t BMx280_SetConfig(BMx280_HandleTypeDef *bmx280, BMx280_Config config);
uint8_t BMx280_GetOversampling(BMx280_HandleTypeDef *bmx280, BMx280_Oversampling *oversampling);
uint8_t BMx280_SetOversampling(BMx280_HandleTypeDef *bmx280, BMx280_Oversampling oversampling);
uint8_t BMx280_GetMode(BMx280_HandleTypeDef *bmx280, BMx280_MODE *mode);
uint8_t BMx280_SetMode(BMx280_HandleTypeDef *bmx280, BMx280_MODE mode);
uint8_t BMx280_ReadMeasurements(BMx280_HandleTypeDef *bmx280);
float BMx280_GetTemperature(BMx280_HandleTypeDef *bmx280);
float BMx280_GetPressure(BMx280_HandleTypeDef *bmx280);
float BMx280_GetHumidity(BMx280_HandleTypeDef *bmx280);

#ifdef __cplusplus
}
#endif


#endif /* INC_BMX280_H_ */
