# Serra Rocketry libs

Biblioteca de drivers de sensores de telemetria para **ESP32**, organizada no
formato de **componentes do ESP-IDF** e com exemplos executáveis via
**PlatformIO**. O objetivo é isolar a lógica de cada sensor em um componente
reutilizável, deixando os exemplos responsáveis apenas por montar o barramento e
chamar a API do driver.

## Arquitetura

O projeto separa claramente **quem cria o barramento** de **quem conversa com o
sensor**:

```
┌──────────────────────────────────────────────┐
│ examples/test_lsm6ds3                         │  ← aplicação/exemplo
│  • configura pinos, clock e o barramento I2C  │
│  • registra o dispositivo no barramento       │
│  • chama a API do driver em loop              │
└───────────────────────┬───────────────────────┘
                        │ inclui lsm6ds3.h e passa o handle I2C
                        ▼
┌──────────────────────────────────────────────┐
│ components/sensor_lsm6ds3                     │  ← driver do sensor
│  • lsm6ds3_init()                             │
│  • lsm6ds3_read_accel_raw() / _gyro_raw()     │
│  • regras de negócio do sensor (WHO_AM_I,     │
│    registradores, escalas, little-endian)     │
└───────────────────────┬───────────────────────┘
                        │ usa i2c_master_transmit / _transmit_receive
                        ▼
┌──────────────────────────────────────────────┐
│ driver/i2c_master.h (ESP-IDF)                 │  ← barramento
│  i2c_new_master_bus / i2c_master_bus_add_device│
└───────────────────────┬───────────────────────┘
                        ▼
                    Sensor LSM6DS3
```

**Decisões de arquitetura**

- **O driver não cria o barramento.** O handle `i2c_master_dev_handle_t` é
  recebido como parâmetro. Isso torna o componente independente de porta I2C,
  pinos e velocidade, e fácil de testar com um barramento já existente.
- **Acesso direto ao `i2c_master` do ESP-IDF**, sem camada de abstração
  intermediária. Menos código, menos indireção e uso pleno dos recursos do
  driver oficial.
- **Um componente por sensor**, com prefixo no nome do diretório
  (`sensor_<modelo>`) e nos símbolos públicos (`lsm6ds3_*`), evitando colisão
  entre drivers.
- **Exemplos isolados**: cada exemplo é um projeto ESP-IDF completo, com seu
  próprio `CMakeLists.txt` e `platformio.ini`, e aponta para `components/` via
  `EXTRA_COMPONENT_DIRS`.

## Estrutura de diretórios

```
esp-telemetry-drivers/
├── components/                     # Componentes do ESP-IDF (bibliotecas)
│   └── sensor_lsm6ds3/
│       ├── CMakeLists.txt          # idf_component_register(...)
│       ├── include/
│       │   └── lsm6ds3.h           # API pública, registradores e macros
│       └── lsm6ds3.c               # Implementação (I2C + regras do sensor)
│
├── examples/                       # Projetos de teste (validação em hardware)
│   └── test_lsm6ds3/
│       ├── CMakeLists.txt          # Projeto ESP-IDF; expõe ../../components
│       ├── platformio.ini          # Ambiente de build/flash
│       └── main/
│           ├── CMakeLists.txt      # Registra main.c e REQUIRES o driver
│           └── main.c              # Monta o I2C e lê o sensor em loop
│
├── platformio.ini                  # Raiz reconhecida pelo PlatformIO
├── .gitignore
└── README.md
```

## Componente `sensor_lsm6ds3`

O componente encapsula tudo que é específico do LSM6DS3:

- endereço I2C (0x6A/0x6B, conforme o pino `SA0`);
- verificação de identidade via `WHO_AM_I` (esperado `0x6A`);
- configuração do acelerômetro (`CTRL1_XL`, ±2 g @ 104 Hz) e do giroscópio
  (`CTRL2_G`, ±245 dps @ 104 Hz);
- leitura de 6 bytes por eixo e montagem little-endian;
- constantes de sensibilidade para conversão das leituras cruas.

O `CMakeLists.txt` do componente resolve a dependência do driver de barramento:

```cmake
idf_component_register(
    SRCS "lsm6ds3.c"
    INCLUDE_DIRS "include"
    REQUIRES driver
)
```

### API pública

| Símbolo                            | Descrição                                                                                                                 |
| ---------------------------------- | ------------------------------------------------------------------------------------------------------------------------- |
| `lsm6ds3_init(dev)`                | Verifica o `WHO_AM_I` e configura os registradores. Retorna `ESP_OK`, `ESP_FAIL` (sensor não reconhecido) ou erro de I2C. |
| `lsm6ds3_read_accel_raw(dev, out)` | Lê aceleração crua nos eixos X/Y/Z.                                                                                       |
| `lsm6ds3_read_gyro_raw(dev, out)`  | Lê velocidade angular crua nos eixos X/Y/Z.                                                                               |
| `lsm6ds3_vec3_t`                   | Struct com `int16_t x, y, z`.                                                                                             |
| `LSM6DS3_ACCEL_MG_PER_LSB`         | 0.061 mg/LSB (±2 g).                                                                                                      |
| `LSM6DS3_GYRO_MDPS_PER_LSB`        | 8.75 mdps/LSB (±245 dps).                                                                                                 |
| `LSM6DS3_I2C_ADDR_SA0_LOW/HIGH`    | Endereços I2C de 7 bits.                                                                                                  |

## Como compilar e gravar

Cada exemplo é compilado a partir da sua própria pasta (o PlatformIO baixa e
gerencia o toolchain e o ESP-IDF):

```bash
# compilar
pio run -d examples/test_lsm6ds3

# gravar na placa
pio run -d examples/test_lsm6ds3 -t upload

# abrir o monitor serial (115200 baud)
pio device monitor -d examples/test_lsm6ds3
```

Antes de gravar, ajuste conforme o hardware:

- **`main/main.c`**: `I2C_MASTER_SCL_IO`, `I2C_MASTER_SDA_IO` e a frequência;
- **`platformio.ini`**: `board` (placa) e `upload_port`, se necessário;
- **endereço I2C**: use `LSM6DS3_I2C_ADDR_SA0_LOW` caso o `SA0` do sensor esteja
  ligado ao GND.

## Como adicionar um novo sensor

1. Crie `components/sensor_<modelo>/` com `include/<modelo>.h`,
   `<modelo>.c` e `CMakeLists.txt`.
2. Exponha uma API pública com prefixo do modelo (ex.: `bme280_init`).
3. No `idf_component_register`, liste `REQUIRES driver`.
4. Crie `examples/test_<modelo>/` copiando a estrutura de um exemplo existente e
   ajustando `REQUIRES` no `main/CMakeLists.txt`.
5. Documente os registradores, escalas e o endereço I2C no cabeçalho.

## Convenções

- Diretórios de componente em minúsculas: `sensor_lsm6ds3`, não
  `sensor_LSM6DS3`.
- Símbolos públicos prefixados com o nome do sensor (`lsm6ds3_*`).
- Um único cabeçalho público por componente, em `include/`.
- Funções retornam `esp_err_t`; a aplicação decide como tratar o erro.
- Artefatos de build (`.pio/`, `build/`, `sdkconfig.*`) não são versionados.
