#include "driver/i2c_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lsm6ds3.h"
#include <stdio.h>

/* Pinos do barramento I2C no ESP32. Ajuste conforme a sua placa. */
#define I2C_MASTER_SCL_IO 9
#define I2C_MASTER_SDA_IO 8
#define I2C_MASTER_FREQ_HZ 100000 /* 100 kHz */

void app_main(void) {
  /* 1. Configuração física do barramento I2C. */
  i2c_master_bus_config_t bus_config = {
      .clk_source = I2C_CLK_SRC_DEFAULT,
      .i2c_port = I2C_NUM_0,
      .scl_io_num = I2C_MASTER_SCL_IO,
      .sda_io_num = I2C_MASTER_SDA_IO,
      .glitch_ignore_cnt = 7,
      .flags.enable_internal_pullup = true,
  };

  i2c_master_bus_handle_t bus_handle;
  ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, &bus_handle));

  /* 2. Registra o LSM6DS3 no barramento. */
  i2c_device_config_t dev_config = {
      .dev_addr_length = I2C_ADDR_BIT_LEN_7,
      .device_address = LSM6DS3_I2C_ADDR_DEFAULT,
      .scl_speed_hz = I2C_MASTER_FREQ_HZ,
  };

  i2c_master_dev_handle_t sensor_handle;
  ESP_ERROR_CHECK(
      i2c_master_bus_add_device(bus_handle, &dev_config, &sensor_handle));

  /* 3. Inicialização lógica do sensor. */
  if (lsm6ds3_init(sensor_handle) != ESP_OK) {
    printf("FALHA: Sensor LSM6DS3 não respondeu!\n");
    return;
  }
  printf("LSM6DS3 inicializado com sucesso!\n");

  /* 4. Laço de leitura. */
  lsm6ds3_vec3_t accel;
  lsm6ds3_vec3_t gyro;
  while (1) {
    if (lsm6ds3_read_accel_raw(sensor_handle, &accel) == ESP_OK &&
        lsm6ds3_read_gyro_raw(sensor_handle, &gyro) == ESP_OK) {
      printf("ACC [mg]: %6.1f %6.1f %6.1f | GYR [dps]: %6.1f %6.1f %6.1f\n",
             accel.x * LSM6DS3_ACCEL_MG_PER_LSB,
             accel.y * LSM6DS3_ACCEL_MG_PER_LSB,
             accel.z * LSM6DS3_ACCEL_MG_PER_LSB,
             gyro.x * LSM6DS3_GYRO_MDPS_PER_LSB / 1000.0f,
             gyro.y * LSM6DS3_GYRO_MDPS_PER_LSB / 1000.0f,
             gyro.z * LSM6DS3_GYRO_MDPS_PER_LSB / 1000.0f);
    } else {
      printf("ERRO: Falha na leitura I2C\n");
    }

    vTaskDelay(pdMS_TO_TICKS(100));
  }
}
