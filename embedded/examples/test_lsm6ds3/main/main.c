#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lsm6ds3.h"
#include <stdbool.h>
#include <stdio.h>

/* Pinos do barramento I2C no ESP32. Ajuste conforme a sua placa. */
#define I2C_MASTER_SCL_IO 9
#define I2C_MASTER_SDA_IO 8
#define I2C_MASTER_FREQ_HZ 100000 /* 100 kHz */

/* Varredura de diagnóstico do barramento. Desligada (0), o boot faz apenas a
 * leitura de WHO_AM_I, igual ao caminho normal. Ligue (1) só para depurar. */
#define I2C_SCAN_ENABLED 0

#if I2C_SCAN_ENABLED
/* Varre o barramento e imprime os endereços que deram ACK (0x03..0x77). */
static void i2c_scan(i2c_master_bus_handle_t bus) {
  printf("Varredura I2C:");
  for (uint8_t addr = 0x03; addr < 0x78; ++addr) {
    if (i2c_master_probe(bus, addr, 20) == ESP_OK) {
      printf(" 0x%02X", addr);
    }
  }
  printf("\n");
}
#endif

/* Mede o nivel idle das linhas com pull-up interno (1=alto/ok, 0=preso). */
static void i2c_check_lines(void) {
  gpio_config_t cfg = {
      .pin_bit_mask = (1ULL << I2C_MASTER_SDA_IO) |
                      (1ULL << I2C_MASTER_SCL_IO),
      .mode = GPIO_MODE_INPUT,
      .pull_up_en = GPIO_PULLUP_ENABLE,
      .pull_down_en = GPIO_PULLDOWN_DISABLE,
      .intr_type = GPIO_INTR_DISABLE,
  };
  gpio_config(&cfg);
  vTaskDelay(pdMS_TO_TICKS(2));
  printf("Niveis idle: SDA(GPIO%d)=%d SCL(GPIO%d)=%d (esperado 1/1)\n",
         I2C_MASTER_SDA_IO, gpio_get_level(I2C_MASTER_SDA_IO),
         I2C_MASTER_SCL_IO, gpio_get_level(I2C_MASTER_SCL_IO));
  gpio_reset_pin(I2C_MASTER_SDA_IO);
  gpio_reset_pin(I2C_MASTER_SCL_IO);
}

/* Lê acelerômetro e giroscópio com algumas retentativas, para absorver NACKs
 * intermitentes de um barramento ainda marginal (o correto é pull-ups fortes). */
static bool lsm6ds3_read_all(i2c_master_dev_handle_t dev, lsm6ds3_vec3_t *accel,
                             lsm6ds3_vec3_t *gyro) {
  for (int attempt = 0; attempt < 3; ++attempt) {
    if (lsm6ds3_read_accel_raw(dev, accel) == ESP_OK &&
        lsm6ds3_read_gyro_raw(dev, gyro) == ESP_OK) {
      return true;
    }
    vTaskDelay(pdMS_TO_TICKS(2));
  }
  return false;
}

void app_main(void) {
  /* 0. Diagnostico eletrico das linhas antes de montar o barramento. */
  i2c_check_lines();

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

  /* 1b. Libera um barramento eventualmente preso por transação abortada. */
  ESP_ERROR_CHECK(i2c_master_bus_reset(bus_handle));

  /* 1c. Varredura de diagnóstico (opcional; ver I2C_SCAN_ENABLED). */
#if I2C_SCAN_ENABLED
  i2c_scan(bus_handle);
#endif

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
    printf("FALHA: LSM6DS3 não inicializou (veja o log acima: endereço I2C "
           "0x%02X e WHO_AM_I).\n",
           LSM6DS3_I2C_ADDR_DEFAULT);
    return;
  }
  printf("LSM6DS3 inicializado com sucesso!\n");

  /* 4. Laço de leitura. */
  lsm6ds3_vec3_t accel;
  lsm6ds3_vec3_t gyro;
  while (1) {
    if (lsm6ds3_read_all(sensor_handle, &accel, &gyro)) {
      printf("ACC [g]: %6.2f %6.2f %6.2f | GYR [dps]: %6.1f %6.1f %6.1f\n",
             accel.x * LSM6DS3_ACCEL_MG_PER_LSB / 1000.0f,
             accel.y * LSM6DS3_ACCEL_MG_PER_LSB / 1000.0f,
             accel.z * LSM6DS3_ACCEL_MG_PER_LSB / 1000.0f,
             gyro.x * LSM6DS3_GYRO_MDPS_PER_LSB / 1000.0f,
             gyro.y * LSM6DS3_GYRO_MDPS_PER_LSB / 1000.0f,
             gyro.z * LSM6DS3_GYRO_MDPS_PER_LSB / 1000.0f);
    } else {
      printf("ERRO: Falha na leitura I2C (3 tentativas)\n");
    }

    vTaskDelay(pdMS_TO_TICKS(100));
  }
}
