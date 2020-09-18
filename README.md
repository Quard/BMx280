# BMx280

Bosch BME280/BMP280 pressure sensor light library. In original adopted for SMT32 Low Layer library but could be easily changed to any other platform.

[BME280 datasheet](https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bme280-ds002.pdf)

[BMP280 datasheet](https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmp280-ds001.pdf)

## How to use

```C
BMx280_HandleTypeDef bme280;

bme280.hi2c = I2C1;
bme280.address = 0x76;

BMx280_Config config = {
    .filter = BMx280_FILTER_4,
    .t_sb = BMx280_TIMESTBY_1000
};

BMx280_Oversampling oversampling = {
    .osrs_t = BMx280_OVERSAMPLING_4,
    .osrs_p = BMx280_OVERSAMPLING_2,
    .osrs_h = BMx280_OVERSAMPLING_1,
};

BMx280_Init(&bme280);
BMx280_SetConfig(&bme280, config);
BMx280_SetOversampling(&bme280, oversampling);
BMx280_SetMode(&bme280, BMx280_MODE_NORMAL);

BMx280_ReadMeasurements(&bme280);

float temperature = BMx280_GetTemperature(&bme280);
float pressure = BMx280_GetPressure(&bme280);
float humidity = BMx280_GetHumidity(&bme280);
```

## Porting routine

1. Change includes in `bmx280.h` to proper for your MCU, also maybe you want to replace I2C with SPI
2. Change `_BMx280_IO_Write8` and `_BMx280_IO_Read` functions in `bmx280.c` to implement read and write functionality according your MCU

## Tested using

sensors:
* GY-BME280 (BME280 sensor)
* GY-91 (BMP280 sensor)

on boards:
* STM32 Nucleo-L073RZ
* STM32 L010F4