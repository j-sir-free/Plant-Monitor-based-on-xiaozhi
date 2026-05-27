#include "plant_monitor.h"
#include "board.h"
#include <esp_log.h>
#include <esp_timer.h>
#include <esp_rom_sys.h>
#include <cJSON.h>
#include <cmath>
// GPIO3 -> ADC1_CH2 -> ADC_CHANNEL_2
#define LIGHT_SENSOR_ADC_CHANNEL ADC_CHANNEL_2
#define LIGHT_SENSOR_ADC_UNIT    ADC_UNIT_1
#define TAG "PlantMonitor"
// ==================== DHT11 温湿度传感器驱动 ====================
// 完全匹配 普中科技 ESP32-S3 DHT11 参考实现
// GPIO模式: INPUT_OUTPUT_OD + 内部上拉，无需在输入/输出间切换

// IO操作宏（与参考代码一致）
#define DHT11_DQ_IN    gpio_get_level(DHT11_DATA_PIN)
#define DHT11_DQ_OUT(x) gpio_set_level(DHT11_DATA_PIN, (x))

static void Dht11PinInit(gpio_num_t gpio) {
    gpio_config_t cfg = {
        .pin_bit_mask = (1ULL << gpio),
        .mode = GPIO_MODE_INPUT_OUTPUT_OD,  // 开漏输入输出
        .pull_up_en = GPIO_PULLUP_ENABLE,   // 内部上拉
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&cfg);
}

// 复位 DHT11: 拉低≥18ms → 拉高30us
static void dht11_reset(void) {
    DHT11_DQ_OUT(0);
    vTaskDelay(pdMS_TO_TICKS(25));
    DHT11_DQ_OUT(1);
    esp_rom_delay_us(30);
}

// 等待 DHT11 应答: 返回0=正常, 1=异常
static uint8_t dht11_check(void) {
    uint8_t retry = 0;
    // DHT11 会拉低 40~80us
    while (DHT11_DQ_IN && retry < 100) {
        retry++;
        esp_rom_delay_us(1);
    }
    if (retry >= 100) return 1;

    // DHT11 拉低后会再次拉高 40~80us
    retry = 0;
    while (!DHT11_DQ_IN && retry < 100) {
        retry++;
        esp_rom_delay_us(1);
    }
    return (retry >= 100) ? 1 : 0;
}

// 读取 1 个 bit
static uint8_t dht11_read_bit(void) {
    uint8_t retry = 0;
    // 等待低电平结束
    while (DHT11_DQ_IN && retry < 100) { retry++; esp_rom_delay_us(1); }
    // 等待高电平开始
    retry = 0;
    while (!DHT11_DQ_IN && retry < 100) { retry++; esp_rom_delay_us(1); }
    // 延时 40us 后采样
    esp_rom_delay_us(40);
    return DHT11_DQ_IN ? 1 : 0;
}

// 读取 1 个字节（高位在前）
static uint8_t dht11_read_byte(void) {
    uint8_t i, data = 0;
    for (i = 0; i < 8; i++) {
        data <<= 1;
        data |= dht11_read_bit();
    }
    return data;
}
PlantMonitor::PlantMonitor() {
    pcf8574_output_state_ = 0xFF;  // PCF8574 初始输出高电平（继电器低电平触发时关闭）
}
PlantMonitor::~PlantMonitor() {
    if (update_timer_) {
        esp_timer_stop(update_timer_);
        esp_timer_delete(update_timer_);
    }
    if (init_delay_timer_) {
        esp_timer_stop(init_delay_timer_);
        esp_timer_delete(init_delay_timer_);
    }
    if (adc_handle_) {
        adc_oneshot_del_unit(adc_handle_);
    }
}
// ==================== PCF8574 GPIO 位模拟 I2C 驱动 ====================
// 通过 GPIO4(SDA) 和 GPIO5(SCL) 位模拟实现 I2C 通信
// 与摄像头 SCCB 共享引脚但独立操作，不依赖任何 I2C 驱动
#define PCF8574_SDA_PIN   GPIO_NUM_4
#define PCF8574_SCL_PIN   GPIO_NUM_5
#define I2C_DELAY_US      5
// 初始化 SDA 引脚（开漏模拟：输出0拉低，输出1时切换到输入=释放）
static void Pcf8574SdaSetOutput(void) {
    gpio_config_t cfg = {
        .pin_bit_mask = (1ULL << PCF8574_SDA_PIN),
        .mode = GPIO_MODE_OUTPUT_OD,   // 开漏输出, 符合I2C规范
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&cfg);
}
static void Pcf8574SdaSetInput(void) {
    gpio_config_t cfg = {
        .pin_bit_mask = (1ULL << PCF8574_SDA_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&cfg);
}
static void Pcf8574SclInit(void) {
    gpio_config_t cfg = {
        .pin_bit_mask = (1ULL << PCF8574_SCL_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&cfg);
    gpio_set_level(PCF8574_SCL_PIN, 1);
}
static void I2cDelay(void) {
    esp_rom_delay_us(I2C_DELAY_US);
}
static void I2cStart(void) {
    Pcf8574SdaSetOutput();
    gpio_set_level(PCF8574_SDA_PIN, 1);
    gpio_set_level(PCF8574_SCL_PIN, 1);
    I2cDelay();
    gpio_set_level(PCF8574_SDA_PIN, 0);
    I2cDelay();
    gpio_set_level(PCF8574_SCL_PIN, 0);
}
static void I2cStop(void) {
    Pcf8574SdaSetOutput();
    gpio_set_level(PCF8574_SDA_PIN, 0);
    gpio_set_level(PCF8574_SCL_PIN, 1);
    I2cDelay();
    gpio_set_level(PCF8574_SDA_PIN, 1);
    I2cDelay();
    gpio_set_level(PCF8574_SCL_PIN, 0);
}
// 发送一个字节，返回是否收到 ACK
static bool I2cWriteByte(uint8_t data) {
    Pcf8574SdaSetOutput();
    for (int i = 0; i < 8; i++) {
        if (data & 0x80)
            gpio_set_level(PCF8574_SDA_PIN, 1);
        else
            gpio_set_level(PCF8574_SDA_PIN, 0);
        I2cDelay();
        gpio_set_level(PCF8574_SCL_PIN, 1);
        I2cDelay();
        gpio_set_level(PCF8574_SCL_PIN, 0);
        data <<= 1;
    }
    // 释放 SDA 并读取 ACK
    Pcf8574SdaSetInput();
    gpio_set_level(PCF8574_SCL_PIN, 1);
    I2cDelay();
    bool ack = (gpio_get_level(PCF8574_SDA_PIN) == 0);
    gpio_set_level(PCF8574_SCL_PIN, 0);
    return ack;
}
// 读取一个字节，ack=true 发送 ACK，ack=false 发送 NACK
static uint8_t I2cReadByte(bool ack) {
    uint8_t data = 0;
    Pcf8574SdaSetInput();
    for (int i = 0; i < 8; i++) {
        gpio_set_level(PCF8574_SCL_PIN, 1);
        I2cDelay();
        data = (data << 1) | (gpio_get_level(PCF8574_SDA_PIN) ? 1 : 0);
        gpio_set_level(PCF8574_SCL_PIN, 0);
        I2cDelay();
    }
    // 发送 ACK / NACK
    Pcf8574SdaSetOutput();
    gpio_set_level(PCF8574_SDA_PIN, ack ? 0 : 1);
    I2cDelay();
    gpio_set_level(PCF8574_SCL_PIN, 1);
    I2cDelay();
    gpio_set_level(PCF8574_SCL_PIN, 0);
    Pcf8574SdaSetInput();
    return data;
}
bool PlantMonitor::Pcf8574Init() {
    // 初始化 GPIO 引脚用于位模拟 I2C
    Pcf8574SclInit();
    Pcf8574SdaSetInput();
    // 发送探测信号检测 PCF8574 是否存在
    I2cStart();
    bool ack = I2cWriteByte((PCF8574_I2C_ADDR << 1) | 0x00); // 写地址
    I2cStop();
    if (!ack) {
        ESP_LOGW(TAG, "PCF8574未检测到 (地址0x%02X)，请检查接线", PCF8574_I2C_ADDR);
        return false;
    }
    // 初始化所有输出为高电平（继电器模块通常低电平触发，高电平=关闭）
    if (!Pcf8574Write(0xFF)) {
        return false;
    }
    ESP_LOGI(TAG, "PCF8574初始化成功 (GPIO位模拟I2C, 地址0x%02X)", PCF8574_I2C_ADDR);
    return true;
}
bool PlantMonitor::Pcf8574Write(uint8_t data) {
    I2cStart();
    bool ack1 = I2cWriteByte((PCF8574_I2C_ADDR << 1) | 0x00);
    bool ack2 = I2cWriteByte(data);
    I2cStop();
    return ack1 && ack2;
}
uint8_t PlantMonitor::Pcf8574Read() {
    uint8_t data = 0xFF;
    I2cStart();
    if (I2cWriteByte((PCF8574_I2C_ADDR << 1) | 0x01)) {
        data = I2cReadByte(false); // NACK after last byte
    }
    I2cStop();
    return data;
}
// ==================== DHT11 读取实现 ====================
// 完全匹配 普中科技 dht11_read_data 参考实现

bool PlantMonitor::Dht11Read(float& temperature, float& humidity) {
    uint8_t buf[5];
    uint8_t i;

    dht11_reset();
    if (dht11_check() == 0) {
        for (i = 0; i < 5; i++) {
            buf[i] = dht11_read_byte();
        }
        if ((buf[0] + buf[1] + buf[2] + buf[3]) == buf[4]) {
            humidity    = (float)buf[0];
            temperature = (float)buf[2];
            return true;
        }
        ESP_LOGW(TAG, "DHT11 校验和错误");
        return false;
    }
    ESP_LOGW(TAG, "DHT11 无应答");
    return false;
}
// ==================== 光照传感器 ADC 读取 ====================
int PlantMonitor::LightSensorRead() {
    int adc_raw = 0;
    esp_err_t ret = adc_oneshot_read(adc_handle_, LIGHT_SENSOR_ADC_CHANNEL, &adc_raw);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "光照传感器 ADC 读取失败");
        return -1;
    }
    return adc_raw;
}
// ==================== 土壤湿度读取 ====================
int PlantMonitor::SoilMoistureReadDigital() {
    uint8_t input = Pcf8574Read();
    return (input >> PCF8574_PIN_SOIL_DO) & 0x01;
}

// ==================== ADS1115 I2C ADC 驱动 ====================
// 与PCF8574共用位模拟I2C总线 (GPIO4=SDA, GPIO5=SCL)
// ADS1115: 16-bit delta-sigma ADC, I2C地址 0x48 (ADDR=GND)

static bool Ads1115WriteRegister(uint8_t reg, uint16_t value) {
    I2cStart();
    bool ack1 = I2cWriteByte((ADS1115_I2C_ADDR << 1) | 0x00);
    if (!ack1) { I2cStop(); return false; }
    bool ack2 = I2cWriteByte(reg);
    bool ack3 = I2cWriteByte((value >> 8) & 0xFF);
    bool ack4 = I2cWriteByte(value & 0xFF);
    I2cStop();
    return ack1 && ack2 && ack3 && ack4;
}

bool PlantMonitor::Ads1115ReadConversion(int16_t* result) {
    I2cStart();
    bool ack1 = I2cWriteByte((ADS1115_I2C_ADDR << 1) | 0x00);
    if (!ack1) { I2cStop(); return false; }
    bool ack2 = I2cWriteByte(ADS1115_REG_CONVERSION);
    I2cStop();
    if (!ack2) return false;

    I2cStart();
    bool ack3 = I2cWriteByte((ADS1115_I2C_ADDR << 1) | 0x01);
    if (!ack3) { I2cStop(); return false; }
    uint8_t msb = I2cReadByte(true);
    uint8_t lsb = I2cReadByte(false);
    I2cStop();

    *result = ((int16_t)msb << 8) | lsb;
    return true;
}

bool PlantMonitor::Ads1115Init() {
    // I2C总线复位: 确保总线处于干净状态
    I2cStop();
    esp_rom_delay_us(200);

    uint16_t config = (ADS1115_CONFIG_MSB << 8) | ADS1115_CONFIG_LSB;
    if (!Ads1115WriteRegister(ADS1115_REG_CONFIG, config)) {
        ESP_LOGW(TAG, "ADS1115未检测到 (地址0x%02X)，请检查接线", ADS1115_I2C_ADDR);
        return false;
    }
    vTaskDelay(pdMS_TO_TICKS(10));

    int16_t test_val = 0;
    if (!Ads1115ReadConversion(&test_val)) {
        ESP_LOGW(TAG, "ADS1115读取失败");
        return false;
    }

    ESP_LOGI(TAG, "ADS1115初始化成功 (I2C地址0x%02X, 初始值=%d)", ADS1115_I2C_ADDR, test_val);
    return true;
}

void PlantMonitor::SoilMoistureReadAnalog() {
    if (!ads1115_present_) return;

    int16_t raw = 0;
    if (!Ads1115ReadConversion(&raw)) {
        ESP_LOGW(TAG, "ADS1115土壤湿度读取失败");
        return;
    }

    sensor_data_.soil_moisture_raw = (int)raw;

    // 转换为百分比: 0% = 干燥(空气中), 100% = 浸入水中
    int percent = 100 - (raw - SOIL_MOISTURE_WATER_VALUE) * 100
                       / (SOIL_MOISTURE_AIR_VALUE - SOIL_MOISTURE_WATER_VALUE);
    if (percent < 0) percent = 0;
    if (percent > 100) percent = 100;
    sensor_data_.soil_moisture_percent = percent;
}
// ==================== 继电器控制 ====================
void PlantMonitor::SetRelay(int relay_index, bool on) {
    if (!initialized_) return;
    int pin;
    switch (relay_index) {
        case 0: pin = PCF8574_PIN_RELAY_PUMP;   break;
        case 1: pin = PCF8574_PIN_RELAY_LIGHT;  break;
        case 2: pin = PCF8574_PIN_RELAY_HEATER; break;
        default: return;
    }

    sensor_data_.relay_pump   = (relay_index == 0) ? on : sensor_data_.relay_pump;
    sensor_data_.relay_light  = (relay_index == 1) ? on : sensor_data_.relay_light;
    sensor_data_.relay_heater = (relay_index == 2) ? on : sensor_data_.relay_heater;

#ifdef RELAY_ACTIVE_HIGH
    // COM+NC 接线：高电平→继电器OFF→NC闭合→设备ON
    if (on) pcf8574_output_state_ |=  (1 << pin);
    else    pcf8574_output_state_ &= ~(1 << pin);
#else
    // COM+NO 接线：低电平→继电器ON→NO闭合→设备ON
    if (on) pcf8574_output_state_ &= ~(1 << pin);
    else    pcf8574_output_state_ |=  (1 << pin);
#endif
    Pcf8574Write(pcf8574_output_state_);
}
// ==================== 仅读取传感器（初版：不执行自动控制）====================
void PlantMonitor::ReadSensorsForInit() {
    float temp = 0, hum = 0;
    for (int retry = 0; retry < 3; retry++) {
        if (Dht11Read(temp, hum)) {
            sensor_data_.temperature = temp;
            sensor_data_.humidity = hum;
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    int light = LightSensorRead();
    if (light >= 0) {
        sensor_data_.light_value = light;
    }
    sensor_data_.soil_moisture_digital = (SoilMoistureReadDigital() == 1);
    if (ads1115_present_) {
        SoilMoistureReadAnalog();
    } else {
        sensor_data_.soil_moisture_percent = sensor_data_.soil_moisture_digital ? 100 : 0;
    }
    ESP_LOGI(TAG, "初始传感器读数-温度:%.1f℃ 湿度:%.1f%% 光照:%d 土壤:%d%%(raw:%d dig:%d)",
             sensor_data_.temperature, sensor_data_.humidity,
             sensor_data_.light_value, sensor_data_.soil_moisture_percent,
             sensor_data_.soil_moisture_raw, sensor_data_.soil_moisture_digital);
}

// ==================== 自动控制逻辑 ====================
void PlantMonitor::AutoControl() {
    // 温度控制：温度低于下限 -> 开启加热片；高于上限 -> 关闭
    if (sensor_data_.temperature < thresholds_.temp_min) {
        SetRelay(2, true);
    } else if (sensor_data_.temperature > thresholds_.temp_max) {
        SetRelay(2, false);
    }

    // 光照控制
#ifdef LIGHT_SENSOR_INVERTED
    // 光照越强ADC值越低：值高=暗→开灯，值低=亮→关灯
    if (sensor_data_.light_value > thresholds_.light_max && sensor_data_.light_value >= 0) {
        SetRelay(1, true);
    } else if (sensor_data_.light_value < thresholds_.light_min) {
        SetRelay(1, false);
    }
#else
    // 标准传感器：值低=暗→开灯，值高=亮→关灯
    if (sensor_data_.light_value < thresholds_.light_min && sensor_data_.light_value >= 0) {
        SetRelay(1, true);
    } else if (sensor_data_.light_value > thresholds_.light_max) {
        SetRelay(1, false);
    }
#endif

    bool need_water = (sensor_data_.soil_moisture_percent < thresholds_.soil_moisture_dry) ||
                      (sensor_data_.humidity < thresholds_.humidity_min);
    bool water_ok = (sensor_data_.soil_moisture_percent > 70) &&
                    (sensor_data_.humidity > thresholds_.humidity_max);
    if (need_water) {
        SetRelay(0, true);
    } else if (water_ok) {
        SetRelay(0, false);
    }
}
// ==================== 传感器数据更新 ====================
void PlantMonitor::Update() {
    if (!initialized_) return;
    // 读取DHT11（带重试）- 已修复重试间隔为1秒（符合DHT11规范）
    float temp = 0, hum = 0;
    for (int retry = 0; retry < 3; retry++) {
        if (Dht11Read(temp, hum)) {
            sensor_data_.temperature = temp;
            sensor_data_.humidity = hum;
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(1000)); // 强制1秒间隔，符合DHT11 datasheet要求
    }
    // 读取光照传感器
    int light = LightSensorRead();
    if (light >= 0) {
        sensor_data_.light_value = light;
    }
    // 读取土壤湿度（模拟值优先，数字值作后备）
    sensor_data_.soil_moisture_digital = (SoilMoistureReadDigital() == 1);
    if (ads1115_present_) {
        SoilMoistureReadAnalog();
    } else {
        sensor_data_.soil_moisture_percent = sensor_data_.soil_moisture_digital ? 100 : 0;
    }
    // 执行自动控制
    AutoControl();
    ESP_LOGI(TAG, "温度:%.1f℃ 湿度:%.1f%% 光照:%d 土壤:%d%% 水泵:%d 灯光:%d 加热:%d",
             sensor_data_.temperature, sensor_data_.humidity,
             sensor_data_.light_value, sensor_data_.soil_moisture_percent,
             sensor_data_.relay_pump, sensor_data_.relay_light, sensor_data_.relay_heater);
}
// ==================== 阈值设置 ====================
void PlantMonitor::SetThresholds(const SensorThresholds& t) {
    thresholds_ = t;
    ESP_LOGI(TAG, "阈值已更新: T[%d-%d]℃ H[%d-%d]%% L[%d-%d] S[%d%%]",
             thresholds_.temp_min, thresholds_.temp_max,
             thresholds_.humidity_min, thresholds_.humidity_max,
             thresholds_.light_min, thresholds_.light_max,
             thresholds_.soil_moisture_dry);
}
// ==================== MCP 工具注册 ====================
void PlantMonitor::RegisterMcpTools() {
    auto& mcp = McpServer::GetInstance();
    // 获取传感器数据
    mcp.AddTool(
        "plant.get_sensor_data",
        "获取植物生长环境的传感器数据。返回温度、湿度、光照、土壤湿度等当前值。\n"
        "当用户询问植物生长状态、环境参数时使用此工具。\n"
        "花卉养护参考：多数开花植物最适温度15-30℃、湿度40-80%；多肉植物温度10-35℃、湿度30-50%；热带植物温度20-35℃、湿度60-90%。",
        PropertyList(),
        [this](const PropertyList&) -> ReturnValue {
            Update();
            auto& s = sensor_data_;
            char buf[512];
            snprintf(buf, sizeof(buf),
                "{\"temperature\":%.1f,\"humidity\":%.1f,\"light\":%d,"
                "\"soil_moisture_percent\":%d,\"soil_moisture_raw\":%d,"
                "\"soil_moisture_digital\":%s,"
                "\"relay_pump\":%s,\"relay_light\":%s,"
                "\"relay_heater\":%s}",
                s.temperature, s.humidity, s.light_value,
                s.soil_moisture_percent, s.soil_moisture_raw,
                s.soil_moisture_digital ? "true" : "false",
                s.relay_pump ? "true" : "false",
                s.relay_light ? "true" : "false",
                s.relay_heater ? "true" : "false");
            return std::string(buf);
        }
    );
    // 设置温度阈值
    mcp.AddTool(
        "plant.set_temp_threshold",
        "设置温度控制阈值。当温度低于最低阈值时开启加热片，高于最高阈值时关闭。\n"
        "参数：`temp_min`(最低温度℃), `temp_max`(最高温度℃)",
        PropertyList({
            Property("temp_min", kPropertyTypeInteger, 5, 50),
            Property("temp_max", kPropertyTypeInteger, 5, 50),
        }),
        [this](const PropertyList& props) -> ReturnValue {
            thresholds_.temp_min = props["temp_min"].value<int>();
            thresholds_.temp_max = props["temp_max"].value<int>();
            char buf[128];
            snprintf(buf, sizeof(buf),
                "{\"message\":\"温度阈值已设置为%d-%d℃\"}",
                thresholds_.temp_min, thresholds_.temp_max);
            return std::string(buf);
        }
    );
    // 设置湿度阈值
    mcp.AddTool(
        "plant.set_humidity_threshold",
        "设置湿度控制阈值。当湿度低于最低阈值或土壤干燥时开启水泵浇水。\n"
        "参数：`humidity_min`(最低湿度%%), `humidity_max`(最高湿度%%)",
        PropertyList({
            Property("humidity_min", kPropertyTypeInteger, 10, 100),
            Property("humidity_max", kPropertyTypeInteger, 10, 100),
        }),
        [this](const PropertyList& props) -> ReturnValue {
            thresholds_.humidity_min = props["humidity_min"].value<int>();
            thresholds_.humidity_max = props["humidity_max"].value<int>();
            char buf[128];
            snprintf(buf, sizeof(buf),
                "{\"message\":\"湿度阈值已设置为%d-%d%%\"}",
                thresholds_.humidity_min, thresholds_.humidity_max);
            return std::string(buf);
        }
    );
    // 设置光照阈值
    mcp.AddTool(
        "plant.set_light_threshold",
        "设置光照控制阈值。当光照低于最低值时开启补光灯，高于最高值时关闭。\n"
        "参数：`light_min`(最低光照ADC值,0-4095), `light_max`(最高光照ADC值,0-4095)",
        PropertyList({
            Property("light_min", kPropertyTypeInteger, 0, 4095),
            Property("light_max", kPropertyTypeInteger, 0, 4095),
        }),
        [this](const PropertyList& props) -> ReturnValue {
            thresholds_.light_min = props["light_min"].value<int>();
            thresholds_.light_max = props["light_max"].value<int>();
            char buf[128];
            snprintf(buf, sizeof(buf),
                "{\"message\":\"光照阈值已设置为%d-%d\"}",
                thresholds_.light_min, thresholds_.light_max);
            return std::string(buf);
        }
    );
    // 获取当前阈值
    mcp.AddTool(
        "plant.get_thresholds",
        "获取当前所有控制阈值（温度、湿度、光照的上下限）。",
        PropertyList(),
        [this](const PropertyList&) -> ReturnValue {
            char buf[320];
            snprintf(buf, sizeof(buf),
                "{\"temp_min\":%d,\"temp_max\":%d,"
                "\"humidity_min\":%d,\"humidity_max\":%d,"
                "\"light_min\":%d,\"light_max\":%d,"
                "\"soil_moisture_dry\":%d}",
                thresholds_.temp_min, thresholds_.temp_max,
                thresholds_.humidity_min, thresholds_.humidity_max,
                thresholds_.light_min, thresholds_.light_max,
                thresholds_.soil_moisture_dry);
            return std::string(buf);
        }
    );
    // 设置土壤湿度阈值
    mcp.AddTool(
        "plant.set_soil_moisture_threshold",
        "设置土壤湿度浇水阈值。当土壤湿度百分比低于此阈值时开启水泵浇水。\n"
        "参数：`soil_moisture_dry`(干燥阈值%%, 0-100)",
        PropertyList({
            Property("soil_moisture_dry", kPropertyTypeInteger, 0, 100),
        }),
        [this](const PropertyList& props) -> ReturnValue {
            thresholds_.soil_moisture_dry = props["soil_moisture_dry"].value<int>();
            char buf[128];
            snprintf(buf, sizeof(buf),
                "{\"message\":\"土壤湿度阈值已设置为%d%%\"}",
                thresholds_.soil_moisture_dry);
            return std::string(buf);
        }
    );

    // 手动控制水泵
    mcp.AddTool(
        "plant.control_pump",
        "手动控制水泵开关。`on`: true=开启, false=关闭",
        PropertyList({Property("on", kPropertyTypeBoolean)}),
        [this](const PropertyList& props) -> ReturnValue {
            bool on = props["on"].value<bool>();
            SetRelay(0, on);
            return on ? "{\"message\":\"水泵已开启\"}" : "{\"message\":\"水泵已关闭\"}";
        }
    );
    // 手动控制补光灯
    mcp.AddTool(
        "plant.control_light",
        "手动控制补光灯开关。`on`: true=开启, false=关闭",
        PropertyList({Property("on", kPropertyTypeBoolean)}),
        [this](const PropertyList& props) -> ReturnValue {
            bool on = props["on"].value<bool>();
            SetRelay(1, on);
            return on ? "{\"message\":\"补光灯已开启\"}" : "{\"message\":\"补光灯已关闭\"}";
        }
    );
    // 手动控制加热片
    mcp.AddTool(
        "plant.control_heater",
        "手动控制加热片开关。`on`: true=开启, false=关闭",
        PropertyList({Property("on", kPropertyTypeBoolean)}),
        [this](const PropertyList& props) -> ReturnValue {
            bool on = props["on"].value<bool>();
            SetRelay(2, on);
            return on ? "{\"message\":\"加热片已开启\"}" : "{\"message\":\"加热片已关闭\"}";
        }
    );
    // AI综合分析接口：一次性设置所有阈值
    mcp.AddTool(
        "plant.adjust_all_thresholds",
        "根据植物生长阶段或环境变化，一次性调整所有传感器阈值。\n"
        "AI可以通过分析摄像头图像判断植物状态后自动调用。\n"
        "参数：`temp_min`(最低温度), `temp_max`(最高温度), "
        "`humidity_min`(最低湿度), `humidity_max`(最高湿度), "
        "`light_min`(最低光照), `light_max`(最高光照), "
        "`soil_moisture_dry`(土壤干燥阈值%%, 可选, 默认20)",
        PropertyList({
            Property("temp_min", kPropertyTypeInteger, 0, 60),
            Property("temp_max", kPropertyTypeInteger, 0, 60),
            Property("humidity_min", kPropertyTypeInteger, 10, 100),
            Property("humidity_max", kPropertyTypeInteger, 10, 100),
            Property("light_min", kPropertyTypeInteger, 0, 4095),
            Property("light_max", kPropertyTypeInteger, 0, 4095),
            Property("soil_moisture_dry", kPropertyTypeInteger, 0, 100),
        }),
        [this](const PropertyList& props) -> ReturnValue {
            thresholds_.temp_min = props["temp_min"].value<int>();
            thresholds_.temp_max = props["temp_max"].value<int>();
            thresholds_.humidity_min = props["humidity_min"].value<int>();
            thresholds_.humidity_max = props["humidity_max"].value<int>();
            thresholds_.light_min = props["light_min"].value<int>();
            thresholds_.light_max = props["light_max"].value<int>();
            thresholds_.soil_moisture_dry = props["soil_moisture_dry"].value<int>();
            char buf[384];
            snprintf(buf, sizeof(buf),
                "{\"message\":\"所有阈值已更新：温度%d-%d℃, 湿度%d-%d%%, 光照%d-%d, 土壤干燥<%d%%\"}",
                thresholds_.temp_min, thresholds_.temp_max,
                thresholds_.humidity_min, thresholds_.humidity_max,
                thresholds_.light_min, thresholds_.light_max,
                thresholds_.soil_moisture_dry);
            return std::string(buf);
        }
    );
    // 植物环境综合分析：拍照 + 传感器，AI通过图像+数据综合判断
    mcp.AddTool(
        "plant.analyze_environment",
        "读取所有植物生长传感器数据，并捕获摄像头图像供后续分析。\n"
        "返回传感器JSON（温度/湿度/光照/土壤/阈值/继电器状态）。\n"
        "摄像头帧已捕获到缓冲区，AI可随后调用 self.camera.take_photo 获取图像进行视觉分析。\n\n"
        "===== 花卉养护知识库 =====\n"
        "【通用开花植物】最适温度15-30℃，湿度40-80%，光照充足。低于10℃或高于35℃生长受阻。\n"
        "【多肉植物】最适温度10-35℃，湿度30-50%，需少浇水，喜强光长日照。叶片发软=缺水，叶片发黄透明=水多。\n"
        "【热带植物】如兰花、龟背竹：最适温度20-35℃，湿度60-90%，喜散射光。叶尖焦黄=空气太干或光照过强。\n"
        "【草本花卉】如矮牵牛、万寿菊：最适温度15-28℃，湿度40-70%。\n"
        "【蔬菜类】如番茄、辣椒：最适温度20-30℃，湿度50-70%，需充足光照。\n\n"
        "温度判断：<10℃寒冷危险，>38℃高温危险，15-30℃为最佳区间。\n"
        "湿度判断：<30%干燥需喷雾/浇水，40-80%正常，>90%过湿需通风防霉。\n"
        "光照判断(ADC值)：<1000暗需补光，1000-2500散射光适合多数室内植物，>2500强光。\n"
        "土壤判断(百分比)：0%%=完全干燥，100%%=浸入水中。0-20%%=需浇水，20-60%%=偏干但可接受，>60%%=湿润正常。\n"
        "叶片观察（通过self.camera.take_photo）：黄叶=浇水过多或缺肥；卷叶=过干或过热；叶尖焦枯=肥害或湿度太低。\n"
        "AI应根据传感器数值和摄像头图像，综合分析植物状态并主动调整阈值(plant.adjust_all_thresholds)或控制继电器(plant.control_*)。",
        PropertyList(),
        [this](const PropertyList&) -> ReturnValue {
            // 读取最新传感器数据
            Update();
            auto& s = sensor_data_;

            // 捕获摄像头帧（放入buffer，供后续 take_photo 使用）
            auto* camera = Board::GetInstance().GetCamera();
            bool cam_ok = false;
            if (camera != nullptr) {
                cam_ok = camera->Capture();
            }

            char buf[896];
            snprintf(buf, sizeof(buf),
                "{\"sensors\":{"
                "\"temperature\":%.1f,\"humidity\":%.1f,"
                "\"light\":%d,"
                "\"soil_moisture_percent\":%d,\"soil_moisture_raw\":%d,"
                "\"soil_moisture_digital\":%s,"
                "\"relay_pump\":%s,\"relay_light\":%s,\"relay_heater\":%s"
                "},"
                "\"thresholds\":{"
                "\"temp\":[%d,%d],\"humidity\":[%d,%d],\"light\":[%d,%d],"
                "\"soil_moisture_dry\":%d"
                "},"
                "\"camera_captured\":%s,"
                "\"advice\":\"%s\"}",
                s.temperature, s.humidity, s.light_value,
                s.soil_moisture_percent, s.soil_moisture_raw,
                s.soil_moisture_digital ? "true" : "false",
                s.relay_pump ? "true" : "false",
                s.relay_light ? "true" : "false",
                s.relay_heater ? "true" : "false",
                thresholds_.temp_min, thresholds_.temp_max,
                thresholds_.humidity_min, thresholds_.humidity_max,
                thresholds_.light_min, thresholds_.light_max,
                thresholds_.soil_moisture_dry,
                cam_ok ? "true" : "false",
                cam_ok ? "摄像头图像已就绪，可调用self.camera.take_photo获取图像进行视觉分析"
                       : "摄像头不可用，请仅基于传感器数据判断");
            return std::string(buf);
        }
    );
    ESP_LOGI(TAG, "MCP植物监控工具已注册 (11个工具)");
}
// ==================== 初始化 ====================
// 应在摄像头初始化完成后调用（I2C_NUM_0已由SCCB配置）
void PlantMonitor::Initialize() {
    if (initialized_) {
        ESP_LOGW(TAG, "PlantMonitor已经初始化");
        return;
    }
    ESP_LOGI(TAG, "正在初始化植物监控系统...");
    // 1. 初始化PCF8574（依赖摄像头SCCB已配置的I2C_NUM_0）
    if (!Pcf8574Init()) {
        ESP_LOGW(TAG, "PCF8574初始化失败，继电器功能不可用");
    }

    // 2. 初始化ADS1115（与PCF8574共用I2C总线，在PCF8574之后）
    ads1115_present_ = Ads1115Init();

    // 3. 初始化ADC（光照传感器，GPIO3=ADC1_CH2）
    adc_oneshot_unit_init_cfg_t adc_cfg = {
        .unit_id = LIGHT_SENSOR_ADC_UNIT,
        .clk_src = ADC_RTC_CLK_SRC_DEFAULT,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&adc_cfg, &adc_handle_));
    adc_oneshot_chan_cfg_t chan_cfg = {
        .atten = ADC_ATTEN_DB_12,   // 0-3.3V范围 (12dB衰减)
        .bitwidth = ADC_BITWIDTH_12,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle_, LIGHT_SENSOR_ADC_CHANNEL, &chan_cfg));

    // 4. 初始化DHT11 GPIO（开漏输出+上拉=单总线模式）
    Dht11PinInit(DHT11_DATA_PIN);

    // 5. 注册MCP工具
    RegisterMcpTools();

    // 6. 读取一次传感器数据（用于LCD初始显示），不执行自动控制
    ReadSensorsForInit();

    // 7. 创建3秒延迟定时器，到期后启动周期性自动控制
    esp_timer_create_args_t init_delay_args = {
        .callback = [](void* arg) {
            PlantMonitor* self = static_cast<PlantMonitor*>(arg);
            esp_timer_create_args_t timer_args = {
                .callback = [](void* timer_arg) {
                    PlantMonitor* monitor = static_cast<PlantMonitor*>(timer_arg);
                    monitor->Update();
                },
                .arg = self,
                .dispatch_method = ESP_TIMER_TASK,
                .name = "plant_monitor_timer",
                .skip_unhandled_events = false,
            };
            ESP_ERROR_CHECK(esp_timer_create(&timer_args, &self->update_timer_));
            ESP_ERROR_CHECK(esp_timer_start_periodic(self->update_timer_, 5000000));
            self->auto_control_enabled_ = true;
            esp_timer_delete(self->init_delay_timer_);
            self->init_delay_timer_ = nullptr;
            ESP_LOGI(TAG, "自动控制已启动（上电3秒延迟）");
        },
        .arg = this,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "plant_init_delay",
        .skip_unhandled_events = false,
    };
    ESP_ERROR_CHECK(esp_timer_create(&init_delay_args, &init_delay_timer_));
    ESP_ERROR_CHECK(esp_timer_start_once(init_delay_timer_, 3000000));  // 3秒
    initialized_ = true;
    ESP_LOGI(TAG, "植物监控系统初始化完成");
}