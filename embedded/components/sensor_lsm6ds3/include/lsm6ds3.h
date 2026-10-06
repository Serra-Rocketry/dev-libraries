/**
 * @file lsm6ds3.h
 * @brief Driver do IMU LSM6DS3 (acelerômetro + giroscópio) para ESP-IDF.
 *
 * O componente não cria o barramento I2C: a aplicação registra o barramento e
 * o dispositivo com o driver `i2c_master` do ESP-IDF e passa o handle para as
 * funções abaixo. Dessa forma o mesmo driver serve para qualquer porta/pinos.
 */
#ifndef LSM6DS3_H
#define LSM6DS3_H

#include <stdint.h>

#include "driver/i2c_master.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Endereço I2C de 7 bits, selecionado pelo pino SA0 do sensor. */
#define LSM6DS3_I2C_ADDR_SA0_LOW 0x6A
#define LSM6DS3_I2C_ADDR_SA0_HIGH 0x6B
#define LSM6DS3_I2C_ADDR_DEFAULT LSM6DS3_I2C_ADDR_SA0_HIGH

/* Identificação do sensor. */
#define LSM6DS3_REG_WHO_AM_I 0x0F
#define LSM6DS3_WHO_AM_I_LSM6DS3 0x69    /* LSM6DS3 clássico        */
#define LSM6DS3_WHO_AM_I_LSM6DS3TRC 0x6A /* LSM6DS3TR-C / LSM6DSL   */

/* Registradores de configuração e status. */
#define LSM6DS3_REG_CTRL1_XL 0x10 /* Acelerômetro: ODR e fundo de escala. */
#define LSM6DS3_REG_CTRL2_G 0x11  /* Giroscópio: ODR e fundo de escala.    */
#define LSM6DS3_REG_CTRL3_C 0x12  /* BDU e auto-incremento de endereço.    */
#define LSM6DS3_REG_STATUS 0x1E   /* Bits XLDA/GDA indicam dado disponível. */

/* Base dos registradores de saída (6 bytes por eixo, little-endian). */
#define LSM6DS3_REG_OUTX_L_G 0x22
#define LSM6DS3_REG_OUTX_L_A 0x28

/* Valores de configuração aplicados por lsm6ds3_init(). */
#define LSM6DS3_CTRL1_XL_2G_104HZ 0x40    /* ±2 g  @ 104 Hz */
#define LSM6DS3_CTRL2_G_245DPS_104HZ 0x40 /* ±245 dps @ 104 Hz */
#define LSM6DS3_CTRL3_C_BDU_IFINC 0x44    /* BDU=1 e IF_INC=1 */

/* Sensibilidade típica (datasheet), para converter leitura crua em unidade. */
#define LSM6DS3_ACCEL_MG_PER_LSB 0.061f  /* fundo de escala de ±2 g      */
#define LSM6DS3_GYRO_MDPS_PER_LSB 8.75f  /* fundo de escala de ±245 dps  */

/** Vetor de três eixos com a leitura crua (int16_t, como sai do sensor). */
typedef struct {
  int16_t x;
  int16_t y;
  int16_t z;
} lsm6ds3_vec3_t;

/**
 * @brief Verifica o WHO_AM_I e configura acelerômetro e giroscópio.
 *
 * @param dev Handle do dispositivo I2C já adicionado ao barramento.
 * @return ESP_OK em caso de sucesso; ESP_FAIL se o sensor não responder com um
 *         WHO_AM_I reconhecido (0x69 do LSM6DS3 ou 0x6A do LSM6DS3TR-C); ou o
 *         erro do I2C retornado pelo ESP-IDF.
 */

esp_err_t lsm6ds3_write_reg(i2c_master_dev_handle_t dev, uint8_t reg,
                            uint8_t value);

esp_err_t lsm6ds3_init(i2c_master_dev_handle_t dev);

/**
 * @brief Lê as acelerações cruas nos três eixos (registrador OUTX_L_A).
 */
esp_err_t lsm6ds3_read_accel_raw(i2c_master_dev_handle_t dev,
                                 lsm6ds3_vec3_t *out);

/**
 * @brief Lê as velocidades angulares cruas nos três eixos (OUTX_L_G).
 */
esp_err_t lsm6ds3_read_gyro_raw(i2c_master_dev_handle_t dev,
                                lsm6ds3_vec3_t *out);

#ifdef __cplusplus
}
#endif

#endif /* LSM6DS3_H */
