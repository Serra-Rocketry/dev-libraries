#include "lsm6ds3.h"

/* Tempo máximo (ms) de cada transação I2C. -1 usa o timeout padrão do driver. */
#define LSM6DS3_I2C_TIMEOUT_MS (-1)

/** Escreve um único registrador. */
static esp_err_t lsm6ds3_write_reg(i2c_master_dev_handle_t dev, uint8_t reg,
                                   uint8_t value) {
  uint8_t buffer[2] = {reg, value};
  return i2c_master_transmit(dev, buffer, sizeof(buffer), LSM6DS3_I2C_TIMEOUT_MS);
}

/** Lê @p len bytes a partir do registrador @p reg (write-read transaction). */
static esp_err_t lsm6ds3_read_regs(i2c_master_dev_handle_t dev, uint8_t reg,
                                   uint8_t *data, size_t len) {
  return i2c_master_transmit_receive(dev, &reg, 1, data, len,
                                     LSM6DS3_I2C_TIMEOUT_MS);
}

/** Lê seis bytes de saída e monta um vetor little-endian nos três eixos. */
static esp_err_t lsm6ds3_read_vec3(i2c_master_dev_handle_t dev, uint8_t reg,
                                   lsm6ds3_vec3_t *out) {
  uint8_t raw[6] = {0};

  esp_err_t err = lsm6ds3_read_regs(dev, reg, raw, sizeof(raw));
  if (err != ESP_OK) {
    return err;
  }

  out->x = (int16_t)((uint16_t)raw[1] << 8 | raw[0]);
  out->y = (int16_t)((uint16_t)raw[3] << 8 | raw[2]);
  out->z = (int16_t)((uint16_t)raw[5] << 8 | raw[4]);
  return ESP_OK;
}

esp_err_t lsm6ds3_init(i2c_master_dev_handle_t dev) {
  uint8_t who_am_i = 0;

  esp_err_t err = lsm6ds3_read_regs(dev, LSM6DS3_REG_WHO_AM_I, &who_am_i, 1);
  if (err != ESP_OK) {
    return err;
  }
  if (who_am_i != LSM6DS3_WHO_AM_I_VAL) {
    return ESP_FAIL;
  }

  err = lsm6ds3_write_reg(dev, LSM6DS3_REG_CTRL3_C, LSM6DS3_CTRL3_C_BDU_IFINC);
  if (err != ESP_OK) {
    return err;
  }

  err = lsm6ds3_write_reg(dev, LSM6DS3_REG_CTRL1_XL, LSM6DS3_CTRL1_XL_2G_104HZ);
  if (err != ESP_OK) {
    return err;
  }

  return lsm6ds3_write_reg(dev, LSM6DS3_REG_CTRL2_G,
                           LSM6DS3_CTRL2_G_245DPS_104HZ);
}

esp_err_t lsm6ds3_read_accel_raw(i2c_master_dev_handle_t dev,
                                 lsm6ds3_vec3_t *out) {
  return lsm6ds3_read_vec3(dev, LSM6DS3_REG_OUTX_L_A, out);
}

esp_err_t lsm6ds3_read_gyro_raw(i2c_master_dev_handle_t dev,
                                lsm6ds3_vec3_t *out) {
  return lsm6ds3_read_vec3(dev, LSM6DS3_REG_OUTX_L_G, out);
}
