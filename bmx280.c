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

#include "bmx280.h"


uint8_t _BMx280_IO_Write8(BMx280_HandleTypeDef *bmx280, uint8_t register_addr, uint8_t value) {
  uint8_t buf[] = {register_addr, value};
  uint8_t ptr = 0;

  LL_I2C_HandleTransfer(bmx280->hi2c, bmx280->address << 1 | 1, LL_I2C_ADDRSLAVE_7BIT, 2, LL_I2C_MODE_AUTOEND, LL_I2C_GENERATE_START_WRITE);
  while (!LL_I2C_IsActiveFlag_STOP(bmx280->hi2c)) {
    if (LL_I2C_IsActiveFlag_TXIS(bmx280->hi2c)) {
      LL_I2C_TransmitData8(bmx280->hi2c, buf[ptr]);
      ptr++;
    }
  }
  LL_I2C_ClearFlag_STOP(bmx280->hi2c);

  return BMx280_OK;
}

uint8_t _BMx280_IO_Read(BMx280_HandleTypeDef *bmx280, uint8_t register_addr, uint8_t bytes_to_read, uint8_t *value) {
  LL_I2C_HandleTransfer(bmx280->hi2c, bmx280->address << 1 | 1, LL_I2C_ADDRSLAVE_7BIT, 1, LL_I2C_MODE_AUTOEND, LL_I2C_GENERATE_START_WRITE);
  while (!LL_I2C_IsActiveFlag_STOP(bmx280->hi2c)) {
    if (LL_I2C_IsActiveFlag_TXIS(bmx280->hi2c)) {
      LL_I2C_TransmitData8(bmx280->hi2c, register_addr);
    }
  }
  LL_I2C_ClearFlag_STOP(bmx280->hi2c);

  LL_I2C_HandleTransfer(bmx280->hi2c, bmx280->address << 1, LL_I2C_ADDRSLAVE_7BIT, bytes_to_read, LL_I2C_MODE_AUTOEND, LL_I2C_GENERATE_RESTART_7BIT_READ);
  while (!LL_I2C_IsActiveFlag_STOP(bmx280->hi2c)) {
    if (LL_I2C_IsActiveFlag_RXNE(bmx280->hi2c)) {
      *value = LL_I2C_ReceiveData8(bmx280->hi2c);
      value++;
    }
  }
  LL_I2C_ClearFlag_STOP(bmx280->hi2c);

  return BMx280_OK;
}

uint8_t _BMx280_IO_Read8(BMx280_HandleTypeDef *bmx280, uint8_t register_addr, uint8_t *value) {
  return _BMx280_IO_Read(bmx280, register_addr, 1, value);
}

uint8_t _BMx280_IO_Read16(BMx280_HandleTypeDef *bmx280, uint8_t register_addr, uint16_t *value) {
  return _BMx280_IO_Read(bmx280, register_addr, 2, (uint8_t *) value);
}

uint8_t BMx280_GetStatus(BMx280_HandleTypeDef *bmx280, BMx280_Status *status) {
  return _BMx280_IO_Read8(bmx280, BMx280_REG_STATUS, (uint8_t *) status);
}

uint8_t BMx280_GetConfig(BMx280_HandleTypeDef *bmx280, BMx280_Config *config) {
  return _BMx280_IO_Read8(bmx280, BMx280_REG_CONFIG, (uint8_t *) config);
}

uint8_t BMx280_SetConfig(BMx280_HandleTypeDef *bmx280, BMx280_Config config) {
  return _BMx280_IO_Write8(bmx280, BMx280_REG_CONFIG, *((uint8_t *) &config));
}

uint8_t BMx280_GetOversampling(BMx280_HandleTypeDef *bmx280, BMx280_Oversampling *oversampling) {
  uint8_t ctrl;

  _BMx280_IO_Read8(bmx280, BMx280_REG_CTRL_MEAS, &ctrl);
  oversampling->osrs_t = ctrl >> 5;
  oversampling->osrs_p = (ctrl & 0x1C) >> 2;

  _BMx280_IO_Read8(bmx280, BMx280_REG_CTRL_HUM, &ctrl);
  oversampling->osrs_h = ctrl & 0x07;

  return BMx280_OK;
}

uint8_t BMx280_SetOversampling(BMx280_HandleTypeDef *bmx280, BMx280_Oversampling oversampling) {
  uint8_t ctrl;

  _BMx280_IO_Read8(bmx280, BMx280_REG_CTRL_MEAS, &ctrl);  // read data to do not overwrite mode

  _BMx280_IO_Write8(bmx280, BMx280_REG_CTRL_HUM, oversampling.osrs_h);  // first write humidity as it require
                                                                        // write operation on ctrl_meas register to apply

  ctrl |= (oversampling.osrs_t << 5) | (oversampling.osrs_p << 2);
  _BMx280_IO_Write8(bmx280, BMx280_REG_CTRL_MEAS, ctrl);

  return BMx280_OK;
}

uint8_t BMx280_GetMode(BMx280_HandleTypeDef *bmx280, BMx280_MODE *mode) {
  uint8_t data;

  _BMx280_IO_Read8(bmx280, BMx280_REG_CTRL_MEAS, &data);

  *mode = data & 0b11;

  return BMx280_OK;
}

uint8_t BMx280_SetMode(BMx280_HandleTypeDef *bmx280, BMx280_MODE mode) {
  uint8_t ctrl;

  _BMx280_IO_Read8(bmx280, BMx280_REG_CTRL_MEAS, &ctrl);  // read data to do not overwrite oversampling config
  ctrl |= mode;
  _BMx280_IO_Write8(bmx280, BMx280_REG_CTRL_MEAS, ctrl);

  return BMx280_OK;
}

uint8_t BMx280_ReadCalibrationdata(BMx280_HandleTypeDef *bmx280) {
  uint16_t h4, h5;

  _BMx280_IO_Read16(bmx280, BMx280_REG_DIG_T1, &bmx280->calib_data.dig_T1);
  _BMx280_IO_Read16(bmx280, BMx280_REG_DIG_T2, (uint16_t *) &bmx280->calib_data.dig_T2);
  _BMx280_IO_Read16(bmx280, BMx280_REG_DIG_T3, (uint16_t *) &bmx280->calib_data.dig_T3);
  _BMx280_IO_Read16(bmx280, BMx280_REG_DIG_P1, &bmx280->calib_data.dig_P1);
  _BMx280_IO_Read16(bmx280, BMx280_REG_DIG_P2, (uint16_t *) &bmx280->calib_data.dig_P2);
  _BMx280_IO_Read16(bmx280, BMx280_REG_DIG_P3, (uint16_t *) &bmx280->calib_data.dig_P3);
  _BMx280_IO_Read16(bmx280, BMx280_REG_DIG_P4, (uint16_t *) &bmx280->calib_data.dig_P4);
  _BMx280_IO_Read16(bmx280, BMx280_REG_DIG_P5, (uint16_t *) &bmx280->calib_data.dig_P5);
  _BMx280_IO_Read16(bmx280, BMx280_REG_DIG_P6, (uint16_t *) &bmx280->calib_data.dig_P6);
  _BMx280_IO_Read16(bmx280, BMx280_REG_DIG_P7, (uint16_t *) &bmx280->calib_data.dig_P7);
  _BMx280_IO_Read16(bmx280, BMx280_REG_DIG_P8, (uint16_t *) &bmx280->calib_data.dig_P8);
  _BMx280_IO_Read16(bmx280, BMx280_REG_DIG_P9, (uint16_t *) &bmx280->calib_data.dig_P9);
  if (bmx280->chip_id == BMx280_ID_BME280) {
    _BMx280_IO_Read8(bmx280, BMx280_REG_DIG_H1, &bmx280->calib_data.dig_H1);
    _BMx280_IO_Read16(bmx280, BMx280_REG_DIG_H2, (uint16_t *) &bmx280->calib_data.dig_H2);
    _BMx280_IO_Read8(bmx280, BMx280_REG_DIG_H3, &bmx280->calib_data.dig_H3);
    _BMx280_IO_Read16(bmx280, BMx280_REG_DIG_H4, &h4);
    _BMx280_IO_Read16(bmx280, BMx280_REG_DIG_H5, (uint16_t *) &h5);
    _BMx280_IO_Read8(bmx280, BMx280_REG_DIG_H6, (uint8_t *) &bmx280->calib_data.dig_H6);

    bmx280->calib_data.dig_H4 = (h4 & 0x00ff) << 4 | (h4 & 0x0f00) >> 8;
    bmx280->calib_data.dig_H5 = h5 >> 4;
  }

  return BMx280_OK;
}

uint8_t BMx280_Init(BMx280_HandleTypeDef *bmx280) {
  _BMx280_IO_Read8(bmx280, BMx280_REG_ID, &bmx280->chip_id);
  if (bmx280->chip_id != BMx280_ID_BME280 && bmx280->chip_id != BMx280_ID_BMP280) {
    return BMx280_UNKNOWN_CHIP;
  }
  // perform soft-reset to start working
  _BMx280_IO_Write8(bmx280, BMx280_REG_SOFTRESET, BMx280_SOFTRESET_VALUE);
  BMx280_Status status;
  while (BMx280_GetStatus(bmx280, &status) != 0 || status.im_update);
  BMx280_ReadCalibrationdata(bmx280);

  return BMx280_OK;
}

uint8_t BMx280_ReadMeasurements(BMx280_HandleTypeDef *bmx280) {
  uint8_t data[8] = {0};
  uint8_t data_length = 9;  // for BME280

  if (bmx280->chip_id == BMx280_ID_BMP280) {
    data_length = 7;
  }

  LL_mDelay(20);  // for me doesn't work without small delay before read
  _BMx280_IO_Read(bmx280, BMx280_REG_PRESSUREDATA, 9, data);
  bmx280->adc_pressure = data[0] << 12 | data[1] << 4 | data[2] >> 4;
  bmx280->adc_temperature = data[3] << 12 | data[4] << 4 | data[5] >> 4;
  if (bmx280->chip_id == BMx280_ID_BME280) {
    bmx280->adc_humidity = data[6] << 8 | data[7];
  }

  return BMx280_OK;
}

float BMx280_GetTemperature(BMx280_HandleTypeDef *bmx280) {
  int32_t var1, var2;

  var1 = ((((bmx280->adc_temperature >> 3) - ((int32_t) bmx280->calib_data.dig_T1 << 1)))
          * (int32_t) bmx280->calib_data.dig_T2) >> 11;
  var2 = (((((bmx280->adc_temperature >> 4) - (int32_t) bmx280->calib_data.dig_T1)
          * ((bmx280->adc_temperature >> 4) - (int32_t) bmx280->calib_data.dig_T1)) >> 12)
          * (int32_t) bmx280->calib_data.dig_T3) >> 14;

  bmx280->_fine_temperature = var1 + var2;

  return ((bmx280->_fine_temperature * 5 + 128) >> 8) / 100.;
}

float BMx280_GetPressure(BMx280_HandleTypeDef *bmx280) {
  int32_t var1, var2;
  uint32_t p;

  if (!bmx280->_fine_temperature) {
    BMx280_GetTemperature(bmx280);
  }

  var1 = (((int32_t) bmx280->_fine_temperature) >> 1) - 64000;
  var2 = (((var1 >> 2) * (var1 >> 2)) >> 11) * ((int32_t) bmx280->calib_data.dig_P6);
  var2 = var2 + ((var1 * ((int32_t) bmx280->calib_data.dig_P5)) << 1);
  var2 = (var2 >> 2) + (((int32_t) bmx280->calib_data.dig_P4) << 16);
  var1 = (((bmx280->calib_data.dig_P3 * (((var1 >> 2) * (var1 >> 2)) >> 13)) >> 3) + ((((int32_t) bmx280->calib_data.dig_P2) * var1) >> 1)) >> 18;
  var1 = ((((32768 + var1)) * ((int32_t) bmx280->calib_data.dig_P1)) >> 15);
  if (var1 == 0) {
    return 0.;
  }

  p = (((uint32_t) (((int32_t) 1048576) - bmx280->adc_pressure) - (var2 >> 12))) * 3125;
  if (p < 0x80000000) {
    p = (p << 1) / ((uint32_t) var1);
  } else {
    p = (p / (uint32_t) var1) * 2;
  }
  var1 = (((int32_t) bmx280->calib_data.dig_P9) * ((int32_t) (((p >> 3) * (p >> 3)) >> 13))) >> 12;
  var2 = (((int32_t) (p >> 2)) * ((int32_t) bmx280->calib_data.dig_P8)) >> 13;
  p = (uint32_t) ((int32_t)p + ((var1 + var2 + bmx280->calib_data.dig_P7) >> 4));

  return ((float) p) / 100.;
}

float BMx280_GetHumidity(BMx280_HandleTypeDef *bmx280) {
    int32_t v_x1_u32r;

    if (bmx280->chip_id == BMx280_ID_BMP280) {
      return 0;
    }

    if (!bmx280->_fine_temperature) {
      BMx280_GetTemperature(bmx280);
    }

    v_x1_u32r = bmx280->_fine_temperature - (int32_t) 76800;
    v_x1_u32r = ((((bmx280->adc_humidity << 14) - ((int32_t) bmx280->calib_data.dig_H4 << 20)
            - ((int32_t) bmx280->calib_data.dig_H5 * v_x1_u32r)) + (int32_t) 16384) >> 15)
            * (((((((v_x1_u32r * (int32_t) bmx280->calib_data.dig_H6) >> 10)
                    * (((v_x1_u32r * (int32_t) bmx280->calib_data.dig_H3) >> 11)
                            + (int32_t) 32768)) >> 10) + (int32_t) 2097152)
                    * (int32_t) bmx280->calib_data.dig_H2 + 8192) >> 14);
    v_x1_u32r = v_x1_u32r
            - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7)
                    * (int32_t) bmx280->calib_data.dig_H1) >> 4);
    v_x1_u32r = v_x1_u32r < 0 ? 0 : v_x1_u32r;
    v_x1_u32r = v_x1_u32r > 419430400 ? 419430400 : v_x1_u32r;

    return (v_x1_u32r >> 12) / 1024.;
}
