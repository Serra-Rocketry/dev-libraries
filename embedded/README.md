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
- verificação de identidade via `WHO_AM_I` (aceita `0x69` do LSM6DS3 e `0x6A`
  do LSM6DS3TR-C);
- configuração do acelerômetro (`CTRL1_XL`, ±8 g @ 104 Hz) e do giroscópio
  (`CTRL2_G`, ±500 dps @ 104 Hz);
- leitura de 6 bytes por eixo e montagem little-endian;
- constantes de sensibilidade para conversão das leituras cruas.

### Fundos de escala (`CTRL1_XL` / `CTRL2_G`)

Os valores aplicados pelo `lsm6ds3_init()` ficam nas macros de `lsm6ds3.h` e
devem ser combinados com a constante de sensibilidade correspondente:

| Sensor | Macro do registrador | Valor atual | Sensibilidade |
| ------ | -------------------- | ----------- | ------------- |
| Acelerômetro | `LSM6DS3_CTRL1_XL` | `0x4C` → ±8 g @ 104 Hz | `LSM6DS3_ACCEL_MG_PER_LSB = 0.244` |
| Giroscópio | `LSM6DS3_CTRL2_G` | `0x44` → ±500 dps @ 104 Hz | `LSM6DS3_GYRO_MDPS_PER_LSB = 17.50` |

Nos dois registradores os 4 bits superiores são a ODR (taxa de saída) e os bits
`[3:2]` são o fundo de escala (`FS_*`). Exemplos de fundo de escala:

- **Acelerômetro** (`FS_XL[1:0]`): `00` = ±2 g, `10` = ±4 g, `11` = ±8 g,
  `01` = ±16 g → sensibilidades `0.061`, `0.122`, `0.244`, `0.488` mg/LSB.
- **Giroscópio** (`FS_G[1:0]`): `00` = ±245 dps, `01` = ±500 dps,
  `10` = ±1000 dps, `11` = ±2000 dps → sensibilidades `8.75`, `17.50`, `35.0`,
  `70.0` mdps/LSB.

Ao trocar o fundo de escala, **atualize também** a macro de sensibilidade, senão
a conversão para g/dps fica errada. O `CTRL3_C` usa `0x44` (BDU = 1 e
IF_INC = 1), garantindo leitura coerente dos 6 bytes de cada eixo.

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
| `LSM6DS3_ACCEL_MG_PER_LSB`         | 0.244 mg/LSB (±8 g).                                                                                                      |
| `LSM6DS3_GYRO_MDPS_PER_LSB`        | 17.50 mdps/LSB (±500 dps).                                                                                                |
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

## Montagem (hardware)

Ligações mínimas do LSM6DS3 ao ESP32:

| Sensor | ESP32 (padrão do exemplo) |
| ------ | ------------------------- |
| VCC | 3,3 V |
| GND | GND (comum ao ESP32) |
| SDA | GPIO8 |
| SCL | GPIO9 |
| SA0 | define o endereço (`0x6A` = GND, `0x6B` = VCC) |

**Importante — pull-ups externos:** instale resistores de **4,7 kΩ** de SDA→3,3 V
e de SCL→3,3 V. O pull-up interno do ESP32 (~45 kΩ) é fraco e, dependendo do
módulo e do comprimento dos fios, deixa as bordas lentas e causa `I2C
transaction timeout` ou NACKs intermitentes. Muitos módulos já trazem pull-ups a
bordo (tipicamente 4,7–10 kΩ); nesse caso os externos podem ser omitidos.

Sintoma clássico de montagem: se **SDA ou SCL não ficarem em ~3,3 V em repouso**
(por exemplo uma delas em 0 V), nenhuma transação completa e a varredura não
acha nenhum endereço. O exemplo mede o nível idle das linhas no boot
(`Niveis idle: SDA=.. SCL=..`) e tem uma varredura I2C opcional
(`I2C_SCAN_ENABLED`, desligada por padrão) para ajudar nesse diagnóstico.

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
