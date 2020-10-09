/*
 * Licensed to the Apache Software Foundation (ASF) under one
 * or more contributor license agreements.  See the NOTICE file
 * distributed with this work for additional information
 * regarding copyright ownership.  The ASF licenses this file
 * to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance
 * with the License.  You may obtain a copy of the License at
 *
 *  http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing,
 * software distributed under the License is distributed on an
 * "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY
 * KIND, either express or implied.  See the License for the
 * specific language governing permissions and limitations
 * under the License.
 */

#ifndef __ADXL362_PRIV_H__
#define __ADXL362_PRIV_H__

#ifdef __cplusplus
extern "C" {
#endif

#define ADXL362_DEVID_VAL (0xE5)

#define ADXL362_SPI_READ_CMD_BIT      (0x80)
#define ADXL362_SPI_MULTIBYTE_CMD_BIT (0x40)

    
int adxl362_i2c_write8(struct sensor_itf *itf, uint8_t reg, uint8_t value);
int adxl362_i2c_read8(struct sensor_itf *itf, uint8_t reg, uint8_t *value);
int adxl362_i2c_readlen(struct sensor_itf *itf, uint8_t reg, uint8_t *buffer, uint8_t len);

int adxl362_spi_write8(struct sensor_itf *itf, uint8_t reg, uint8_t value);
int adxl362_spi_read8(struct sensor_itf *itf, uint8_t reg, uint8_t *value);
int adxl362_spi_readlen(struct sensor_itf *itf, uint8_t reg, uint8_t *buffer, uint8_t len);

int adxl362_write8(struct sensor_itf *itf, uint8_t reg, uint8_t value);
int adxl362_read8(struct sensor_itf *itf, uint8_t reg, uint8_t *value);
int adxl362_readlen(struct sensor_itf *itf, uint8_t reg, uint8_t *buffer, uint8_t len);
 
#ifdef __cplusplus
}
#endif

#endif /* __ADXL362_PRIV_H__ */
