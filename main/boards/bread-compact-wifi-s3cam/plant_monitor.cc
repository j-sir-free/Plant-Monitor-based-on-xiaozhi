/**
 * @file    plant_monitor.cc
 * @brief   植物生长环境监控系统 — 核心实现
 * @author  JJD-YOURFATHER
 *
 * 本文件实现了植物监控系统的全部核心逻辑:
 *   - DHT11 单总线温度/湿度传感器驱动
 *   - 光照传感器 ADC 读取 (ESP32-S3 内置 ADC1)
 *   - PCF8574 I2C 8-bit GPIO 扩展器 (位模拟 I2C, 继电器控制)
 *   - ADS1115 I2C 16-bit ADC (土壤湿度模拟量采集)
 *   - 自动控制逻辑 (回滞算法, 继电器开关控制)
 *   - 自动AI模式 (异常检测 + 声学回环触发小智AI)
 *   - 11 个 MCP 工具注册 (AI可调用的传感器/控制接口)
 *
 * I2C 总线拓扑:
 *   GPIO4(SDA) ─┬─ PCF8574 (0x20) ─ 继电器控制 + 土壤数字量
 *               └─ ADS1115 (0x48) ─ 土壤模拟量
 *   GPIO5(SCL) ─┴─ 共用时钟, 位模拟 I2C (~100kHz)
 *   注意: 与摄像头 SCCB 共享 GPIO4/5, 但独立操作 (非 ESP-IDF I2C 驱动)
 */
#include "plant_monitor.h"
#include "board.h"
#include "application.h"
#include "auto_announce.h"
#include <esp_log.h>
#include <esp_timer.h>
#include <esp_rom_sys.h>
#include <cJSON.h>
#include <cmath>

// GPIO3 映射: ESP32-S3 ADC1 通道2
#define LIGHT_SENSOR_ADC_CHANNEL ADC_CHANNEL_2
#define LIGHT_SENSOR_ADC_UNIT    ADC_UNIT_1
#define TAG "PlantMonitor"

/* ================================================================
 * DHT11 单总线温湿度传感器驱动
 * ----------------------------------------------------------------
 * 协议: 单总线 (1-Wire), 开漏 + 上拉
 *
 * 通信时序 (由主机发起):
 *   1. 复位:  主机拉低 ≥18ms → 释放 (上拉至高) → 等待 20-40μs
 *   2. 应答:  DHT11 回应 低80μs + 高80μs
 *   3. 数据:  40 bits = 湿度整数(8) + 湿度小数(8) + 温度整数(8) + 温度小数(8) + 校验和(8)
 *   4. 校验:  checksum == (hum_int + hum_dec + temp_int + temp_dec) & 0xFF
 *
 * Bit 编码 (脉宽调制):
 *   - '0': 低50μs + 高26-28μs
 *   - '1': 低50μs + 高70μs
 *   采样点: 在低电平结束后的 ~40μs 处
 *
 * 参考: 普中科技 ESP32-S3 DHT11 实现
 * ================================================================ */

// IO 操作宏: 读数据线电平 / 写数据线电平
#define DHT11_DQ_IN    gpio_get_level(DHT11_DATA_PIN)
#define DHT11_DQ_OUT(x) gpio_set_level(DHT11_DATA_PIN, (x))

/** GPIO 初始化: 开漏输入输出 + 内部上拉 (模拟单总线开漏) */
static void Dht11PinInit(gpio_num_t gpio) {
    gpio_config_t cfg = {
        .pin_bit_mask = (1ULL << gpio),
        .mode = GPIO_MODE_INPUT_OUTPUT_OD,  // 开漏: 输出0=拉低, 输出1=释放由外部上拉至高
        .pull_up_en = GPIO_PULLUP_ENABLE,   // ESP32 内部 ~45kΩ 上拉
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&cfg);
}

/** 复位 DHT11: 主机拉低 ≥18ms (这里用 25ms 确保余量), 然后释放 30μs */
static void dht11_reset(void) {
    DHT11_DQ_OUT(0);
    vTaskDelay(pdMS_TO_TICKS(25));          // 拉低 25ms (>18ms 最小值)
    DHT11_DQ_OUT(1);
    esp_rom_delay_us(30);                   // 释放后等待 30μs
}

/**
 * 等待 DHT11 应答序列: 低80μs → 高80μs
 * @return 0=检测到应答, 1=超时 (传感器未连接或损坏)
 */
static uint8_t dht11_check(void) {
    uint8_t retry = 0;
    // 阶段1: 等待 DHT11 拉低 (持续 40~80μs)
    while (DHT11_DQ_IN && retry < 100) {
        retry++;
        esp_rom_delay_us(1);                // 每次等待 1μs, 最多 100μs
    }
    if (retry >= 100) return 1;             // 超时: DHT11 未响应

    // 阶段2: 等待 DHT11 释放总线 (拉低后再次拉高 40~80μs)
    retry = 0;
    while (!DHT11_DQ_IN && retry < 100) {
        retry++;
        esp_rom_delay_us(1);
    }
    return (retry >= 100) ? 1 : 0;
}

/**
 * 读取 1-bit 数据
 * 算法: 等待低电平结束 → 等待高电平开始 → 延时40μs采样
 * 在 40μs 处: 如果仍为高 → bit=1 (高持续70μs); 如果已变低 → bit=0 (高仅26-28μs)
 */
static uint8_t dht11_read_bit(void) {
    uint8_t retry = 0;
    // 等待当前低电平结束
    while (DHT11_DQ_IN && retry < 100) { retry++; esp_rom_delay_us(1); }
    // 等待高电平开始 (信号线被释放)
    retry = 0;
    while (!DHT11_DQ_IN && retry < 100) { retry++; esp_rom_delay_us(1); }
    // 延时 40μs 后采样: 在这个时间点, bit=0 已经变低, bit=1 仍为高
    esp_rom_delay_us(40);
    return DHT11_DQ_IN ? 1 : 0;
}

/**
 * 读取 1 字节 (8 bits, MSB first)
 * DHT11 以大端序传输每个字节, 逐位左移组装
 */
static uint8_t dht11_read_byte(void) {
    uint8_t i, data = 0;
    for (i = 0; i < 8; i++) {
        data <<= 1;                         // 左移腾出最低位
        data |= dht11_read_bit();           // 填入新读取的 bit
    }
    return data;
}

/* ================================================================
 * PlantMonitor 构造 / 析构
 * ================================================================ */
PlantMonitor::PlantMonitor() {
    pcf8574_output_state_ = 0xFF;           // PCF8574 初始全部输出 HIGH (继电器默认关闭)
}

PlantMonitor::~PlantMonitor() {
    // 安全释放所有定时器和外设资源
    if (update_timer_) {
        esp_timer_stop(update_timer_);
        esp_timer_delete(update_timer_);
    }
    if (init_delay_timer_) {
        esp_timer_stop(init_delay_timer_);
        esp_timer_delete(init_delay_timer_);
    }
    if (auto_ai_timer_) {
        esp_timer_stop(auto_ai_timer_);
        esp_timer_delete(auto_ai_timer_);
    }
    if (adc_handle_) {
        adc_oneshot_del_unit(adc_handle_);
    }
}

/* ================================================================
 * PCF8574 — I2C 位模拟 (Bit-Banged I2C) 驱动
 * ----------------------------------------------------------------
 * 使用 GPIO4 (SDA) + GPIO5 (SCL) 手动控制 I2C 时序, 不依赖 ESP-IDF I2C 驱动
 *
 * 为什么用位模拟而非硬件 I2C?
 *   1. 摄像头 SCCB 已占用 I2C_NUM_0 硬件控制器
 *   2. ESP-IDF 的 I2C 驱动不支持两个独立 master 共用同一总线
 *   3. 位模拟可完全控制时序, 避免与 SCCB 冲突
 *
 * I2C 时序:
 *   起始条件 (START):  SCL=H, SDA=H → SDA: H→L (在SCL高时SDA下降沿)
 *   停止条件 (STOP):   SCL=H, SDA=L → SDA: L→H (在SCL高时SDA上升沿)
 *   数据位:            在SCL低时改变SDA, 在SCL高时采样SDA
 *   ACK:              第9个SCL周期, 从机拉低SDA表示确认
 *
 * I2C_DELAY_US = 5μs → 半周期5μs → 全周期10μs → ~100kHz 时钟
 * ================================================================ */
#define PCF8574_SDA_PIN   GPIO_NUM_4       // I2C 数据线
#define PCF8574_SCL_PIN   GPIO_NUM_5       // I2C 时钟线
#define I2C_DELAY_US      5                // 每位半周期延迟 (μs)

/**
 * SDA 设置为输出模式 (开漏)
 * 开漏模式下: 输出0→拉低引脚, 输出1→释放引脚(高阻态)→上拉电阻拉高
 * 这符合 I2C 规范: 从机可以安全地将 SDA 拉低而不会与主机冲突
 */
static void Pcf8574SdaSetOutput(void) {
    gpio_config_t cfg = {
        .pin_bit_mask = (1ULL << PCF8574_SDA_PIN),
        .mode = GPIO_MODE_OUTPUT_OD,       // 开漏输出, 符合 I2C 电气规范
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&cfg);
}

/** SDA 设置为输入模式 (用于读取从机数据和 ACK) */
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

/** SCL 引脚初始化 (推挽输出, 始终由主机控制) */
static void Pcf8574SclInit(void) {
    gpio_config_t cfg = {
        .pin_bit_mask = (1ULL << PCF8574_SCL_PIN),
        .mode = GPIO_MODE_OUTPUT,          // SCL 始终由主机驱动, 可用推挽
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&cfg);
    gpio_set_level(PCF8574_SCL_PIN, 1);    // 初始状态: SCL 高
}

/** I2C 时序延迟 — 控制时钟速度 */
static void I2cDelay(void) {
    esp_rom_delay_us(I2C_DELAY_US);        // 5μs → ~100kHz
}

/**
 * I2C START 条件
 * SDA 从高到低的跳变, 发生在 SCL 为高时
 */
static void I2cStart(void) {
    Pcf8574SdaSetOutput();
    gpio_set_level(PCF8574_SDA_PIN, 1);    // SDA 高
    gpio_set_level(PCF8574_SCL_PIN, 1);    // SCL 高
    I2cDelay();
    gpio_set_level(PCF8574_SDA_PIN, 0);    // SDA↓ → START
    I2cDelay();
    gpio_set_level(PCF8574_SCL_PIN, 0);    // SCL↓ → 准备传输数据
}

/**
 * I2C STOP 条件
 * SDA 从低到高的跳变, 发生在 SCL 为高时
 */
static void I2cStop(void) {
    Pcf8574SdaSetOutput();
    gpio_set_level(PCF8574_SDA_PIN, 0);    // SDA 低
    gpio_set_level(PCF8574_SCL_PIN, 1);    // SCL 高
    I2cDelay();
    gpio_set_level(PCF8574_SDA_PIN, 1);    // SDA↑ → STOP
    I2cDelay();
    gpio_set_level(PCF8574_SCL_PIN, 0);    // SCL↓ → 释放总线
}

/**
 * I2C 写一个字节 (MSB first)
 * 算法: 逐位输出 (SCL低时置SDA → SCL高锁存), 8位后读 ACK
 * @return true=收到从机ACK, false=无ACK (从机不存在或忙)
 */
static bool I2cWriteByte(uint8_t data) {
    Pcf8574SdaSetOutput();
    // 逐位输出 8 bits, MSB first
    for (int i = 0; i < 8; i++) {
        if (data & 0x80)
            gpio_set_level(PCF8574_SDA_PIN, 1);  // bit=1 → SDA高
        else
            gpio_set_level(PCF8574_SDA_PIN, 0);  // bit=0 → SDA低
        I2cDelay();
        gpio_set_level(PCF8574_SCL_PIN, 1);      // SCL↑ → 从机锁存数据
        I2cDelay();
        gpio_set_level(PCF8574_SCL_PIN, 0);      // SCL↓ → 准备下一位
        data <<= 1;                                // 移位到下一位
    }
    // 第9个时钟周期: 释放 SDA 并读取从机 ACK
    Pcf8574SdaSetInput();                          // 释放 SDA → 上拉至高
    gpio_set_level(PCF8574_SCL_PIN, 1);            // SCL↑ → 从机输出ACK
    I2cDelay();
    bool ack = (gpio_get_level(PCF8574_SDA_PIN) == 0); // ACK=0 (低电平=确认)
    gpio_set_level(PCF8574_SCL_PIN, 0);            // SCL↓ → 完成 ACK 周期
    return ack;
}

/**
 * I2C 读一个字节 (MSB first)
 * 算法: SDA设为输入, SCL上升沿锁存数据位, 最后发送 ACK/NACK
 * @param ack  true=发送ACK(继续读), false=发送NACK(结束读)
 */
static uint8_t I2cReadByte(bool ack) {
    uint8_t data = 0;
    Pcf8574SdaSetInput();                          // SDA 设为输入
    for (int i = 0; i < 8; i++) {
        gpio_set_level(PCF8574_SCL_PIN, 1);        // SCL↑ → 从机输出下一位
        I2cDelay();
        data = (data << 1) | (gpio_get_level(PCF8574_SDA_PIN) ? 1 : 0);
        gpio_set_level(PCF8574_SCL_PIN, 0);        // SCL↓
        I2cDelay();
    }
    // 第9个时钟周期: 主机发送 ACK (0) 或 NACK (1)
    Pcf8574SdaSetOutput();
    gpio_set_level(PCF8574_SDA_PIN, ack ? 0 : 1);  // ACK=拉低, NACK=释放
    I2cDelay();
    gpio_set_level(PCF8574_SCL_PIN, 1);            // SCL↑ → 锁存ACK/NACK
    I2cDelay();
    gpio_set_level(PCF8574_SCL_PIN, 0);            // SCL↓
    Pcf8574SdaSetInput();                          // 释放SDA, 回到接收状态
    return data;
}

/* ---- PCF8574 高层操作 ---- */

/**
 * PCF8574 初始化
 * 1. 配置 GPIO4/5 用于位模拟 I2C
 * 2. 发送地址探测信号检测芯片是否存在
 * 3. 初始化所有输出为 HIGH (继电器默认关闭)
 */
bool PlantMonitor::Pcf8574Init() {
    Pcf8574SclInit();
    Pcf8574SdaSetInput();

    // 发送 I2C 探测: START → 写地址 → STOP, 检查 ACK
    I2cStart();
    bool ack = I2cWriteByte((PCF8574_I2C_ADDR << 1) | 0x00); // 7位地址左移1位 + R/W=0(写)
    I2cStop();
    if (!ack) {
        ESP_LOGW(TAG, "PCF8574未检测到 (地址0x%02X)，请检查接线", PCF8574_I2C_ADDR);
        return false;
    }

    // 初始化所有输出为 HIGH (RELAY_ACTIVE_HIGH 模式下继电器不吸合)
    if (!Pcf8574Write(0xFF)) {
        return false;
    }
    ESP_LOGI(TAG, "PCF8574初始化成功 (GPIO位模拟I2C, 地址0x%02X)", PCF8574_I2C_ADDR);
    return true;
}

/**
 * 向 PCF8574 写入 8-bit GPIO 输出状态
 * I2C 操作: START → 写地址 → 写数据 → STOP
 */
bool PlantMonitor::Pcf8574Write(uint8_t data) {
    I2cStart();
    bool ack1 = I2cWriteByte((PCF8574_I2C_ADDR << 1) | 0x00);
    bool ack2 = I2cWriteByte(data);
    I2cStop();
    return ack1 && ack2;                       // 两个字节都需收到 ACK
}

/**
 * 读取 PCF8574 8-bit GPIO 输入状态
 * I2C 操作: START → 读地址 → 读数据(NACK) → STOP
 */
uint8_t PlantMonitor::Pcf8574Read() {
    uint8_t data = 0xFF;
    I2cStart();
    if (I2cWriteByte((PCF8574_I2C_ADDR << 1) | 0x01)) { // R/W=1(读)
        data = I2cReadByte(false);                       // NACK: 读取最后一个字节
    }
    I2cStop();
    return data;
}

/* ================================================================
 * DHT11 读取 — 组装完整的读取流程
 * ================================================================ */
bool PlantMonitor::Dht11Read(float& temperature, float& humidity) {
    uint8_t buf[5];                            // 5字节: hum_int, hum_dec, temp_int, temp_dec, checksum
    uint8_t i;

    dht11_reset();                             // 主机发起通信
    if (dht11_check() == 0) {                  // 等待 DHT11 应答
        for (i = 0; i < 5; i++) {
            buf[i] = dht11_read_byte();        // 读取 5 字节 (40 bits)
        }
        // 校验和验证: 前4字节之和低8位 = 第5字节
        if ((buf[0] + buf[1] + buf[2] + buf[3]) == buf[4]) {
            humidity    = (float)buf[0];       // 湿度整数部分 (DHT11 小数始终为0)
            temperature = (float)buf[2];       // 温度整数部分
            return true;
        }
        ESP_LOGW(TAG, "DHT11 校验和错误");
        return false;
    }
    ESP_LOGW(TAG, "DHT11 无应答");
    return false;
}

/* ================================================================
 * 光照传感器 — ESP32-S3 内置 ADC1 读取
 * GPIO3 = ADC1_CH2, 12-bit 分辨率 (0-4095), 12dB 衰减 (0-3.3V量程)
 * ================================================================ */
int PlantMonitor::LightSensorRead() {
    int adc_raw = 0;
    esp_err_t ret = adc_oneshot_read(adc_handle_, LIGHT_SENSOR_ADC_CHANNEL, &adc_raw);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "光照传感器 ADC 读取失败");
        return -1;                             // -1 表示读取失败
    }
    return adc_raw;                            // 0-4095
}

/* ================================================================
 * 土壤湿度数字量 — PCF8574 P4 DO 引脚 (后备方案)
 * ================================================================ */
int PlantMonitor::SoilMoistureReadDigital() {
    uint8_t input = Pcf8574Read();
    return (input >> PCF8574_PIN_SOIL_DO) & 0x01; // 提取 P4 位: 0=干燥, 1=潮湿
}

/* ================================================================
 * ADS1115 — I2C 16-bit ΔΣ ADC 驱动 (土壤湿度模拟量)
 * ----------------------------------------------------------------
 * ADS1115 是 TI 的 4通道 16-bit delta-sigma ADC, 内置 PGA 和电压参考
 *
 * 配置:
 *   AIN0 vs GND 单端输入
 *   PGA = ±4.096V  (1 LSB = 0.125mV)
 *   连续转换模式 (Continuous Conversion)
 *   128 SPS (每次转换 ~7.8ms)
 *
 * I2C 读操作 (2步):
 *   1. 写寄存器指针:  START → dev_addr(W) → 0x00(reg ptr) → STOP
 *   2. 读转换结果:    START → dev_addr(R) → MSB → LSB → STOP
 *
 * I2C 写操作 (配置寄存器, 16-bit):
 *   START → dev_addr(W) → 0x01(reg ptr) → MSB → LSB → STOP
 * ================================================================ */

/**
 * 向 ADS1115 16-bit 寄存器写入数据
 * @param reg   寄存器地址 (0x00=转换结果(只读), 0x01=配置, 0x02=低阈值, 0x03=高阈值)
 * @param value 16-bit 待写入值
 */
static bool Ads1115WriteRegister(uint8_t reg, uint16_t value) {
    I2cStart();
    bool ack1 = I2cWriteByte((ADS1115_I2C_ADDR << 1) | 0x00); // 设备地址 + 写
    if (!ack1) { I2cStop(); return false; }
    bool ack2 = I2cWriteByte(reg);                              // 寄存器指针
    bool ack3 = I2cWriteByte((value >> 8) & 0xFF);              // 数据 MSB
    bool ack4 = I2cWriteByte(value & 0xFF);                     // 数据 LSB
    I2cStop();
    return ack1 && ack2 && ack3 && ack4;
}

/**
 * 读取 ADS1115 16-bit 转换结果
 * 算法: 先写寄存器指针 0x00, 再执行2字节读 (MSB first)
 * @param result 输出参数, 16-bit 有符号 ADC 值
 */
bool PlantMonitor::Ads1115ReadConversion(int16_t* result) {
    // 第1步: 写寄存器指针 (指向转换结果寄存器 0x00)
    I2cStart();
    bool ack1 = I2cWriteByte((ADS1115_I2C_ADDR << 1) | 0x00); // 设备地址 + 写
    if (!ack1) { I2cStop(); return false; }
    bool ack2 = I2cWriteByte(ADS1115_REG_CONVERSION);           // 寄存器指针 = 0x00
    I2cStop();
    if (!ack2) return false;

    // 第2步: 读 2 字节 (MSB + LSB)
    I2cStart();
    bool ack3 = I2cWriteByte((ADS1115_I2C_ADDR << 1) | 0x01); // 设备地址 + 读
    if (!ack3) { I2cStop(); return false; }
    uint8_t msb = I2cReadByte(true);                            // 读 MSB, 发送 ACK
    uint8_t lsb = I2cReadByte(false);                           // 读 LSB, 发送 NACK (结束)
    I2cStop();

    // 组装为有符号 16-bit 整数 (MSB first)
    *result = ((int16_t)msb << 8) | lsb;
    return true;
}

/**
 * ADS1115 初始化: 总线复位 → 写配置 → 验证读取
 * @return true=芯片在线且正常, false=未检测到或通信失败
 */
bool PlantMonitor::Ads1115Init() {
    // 总线复位: 发送STOP确保总线释放在已知状态, 延迟200μs让芯片就绪
    I2cStop();
    esp_rom_delay_us(200);

    // 构造16-bit配置值: MSB << 8 | LSB
    uint16_t config = (ADS1115_CONFIG_MSB << 8) | ADS1115_CONFIG_LSB;

    // 写配置寄存器
    if (!Ads1115WriteRegister(ADS1115_REG_CONFIG, config)) {
        ESP_LOGW(TAG, "ADS1115未检测到 (地址0x%02X)，请检查接线", ADS1115_I2C_ADDR);
        return false;
    }

    // 等待首次转换完成 (128 SPS → 7.8ms, 留10ms余量)
    vTaskDelay(pdMS_TO_TICKS(10));

    // 验证: 读取转换结果, 确认通信正常
    int16_t test_val = 0;
    if (!Ads1115ReadConversion(&test_val)) {
        ESP_LOGW(TAG, "ADS1115读取失败");
        return false;
    }

    ESP_LOGI(TAG, "ADS1115初始化成功 (I2C地址0x%02X, 初始值=%d)", ADS1115_I2C_ADDR, test_val);
    return true;
}

/**
 * 读取土壤湿度模拟量并转换为百分比
 *
 * 传感器特性: 反转型 — 干燥→高电压→高ADC, 浸水→低电压→低ADC
 *
 * 线性映射算法:
 *   DRY_VALUE (空气中)    → 0%
 *   WET_VALUE (浸入水中)  → 100%
 *   中间值按线性插值
 *
 *   公式: percent = 100 - (raw - WET) * 100 / (DRY - WET)
 *   边界: raw ≤ WET → 100%,  raw ≥ DRY → 0%
 */
void PlantMonitor::SoilMoistureReadAnalog() {
    if (!ads1115_present_) return;

    int16_t raw = 0;
    if (!Ads1115ReadConversion(&raw)) {
        ESP_LOGW(TAG, "ADS1115土壤湿度读取失败");
        return;
    }

    sensor_data_.soil_moisture_raw = (int)raw;

    // 反转型传感器线性映射: 干燥(高ADC)→0%, 浸水(低ADC)→100%
    if (raw <= SOIL_MOISTURE_WET_VALUE) {
        sensor_data_.soil_moisture_percent = 100;                  // 饱和
    } else if (raw >= SOIL_MOISTURE_DRY_VALUE) {
        sensor_data_.soil_moisture_percent = 0;                    // 完全干燥
    } else {
        // 线性插值: 从干燥端(高ADC)向湿润端(低ADC)递减
        sensor_data_.soil_moisture_percent = 100 - (raw - SOIL_MOISTURE_WET_VALUE) * 100
                                                   / (SOIL_MOISTURE_DRY_VALUE - SOIL_MOISTURE_WET_VALUE);
    }
}

/* ================================================================
 * 继电器控制
 * ----------------------------------------------------------------
 * RELAY_ACTIVE_HIGH 模式 (COM+NC 接线):
 *   ON  → PCF8574 输出 HIGH → 继电器线圈不吸合 → COM-NC 连通 → 设备通电
 *   OFF → PCF8574 输出 LOW  → 继电器线圈吸合   → COM-NO 连通 → 设备断电
 *
 * 非 RELAY_ACTIVE_HIGH 模式 (COM+NO 接线):
 *   ON  → PCF8574 输出 LOW  → 继电器线圈吸合   → COM-NO 连通 → 设备通电
 *   OFF → PCF8574 输出 HIGH → 继电器线圈不吸合 → COM-NC 连通 → 设备断电
 *
 * 重试机制: I2C 写入可能因总线残留状态失败, 最多重试 3 次, 间隔 500μs
 * ================================================================ */
void PlantMonitor::SetRelay(int relay_index, bool on) {
    if (!initialized_) return;

    // 根据继电器索引查找对应的 PCF8574 引脚
    int pin;
    switch (relay_index) {
        case 0: pin = PCF8574_PIN_RELAY_PUMP;   break;
        case 1: pin = PCF8574_PIN_RELAY_LIGHT;  break;
        case 2: pin = PCF8574_PIN_RELAY_HEATER; break;
        default: return;
    }

    // 更新内存中的传感器状态 (用于 LCD 显示和 API 返回)
    sensor_data_.relay_pump   = (relay_index == 0) ? on : sensor_data_.relay_pump;
    sensor_data_.relay_light  = (relay_index == 1) ? on : sensor_data_.relay_light;
    sensor_data_.relay_heater = (relay_index == 2) ? on : sensor_data_.relay_heater;

    // 更新 PCF8574 输出状态 (bitmask 操作)
#ifdef RELAY_ACTIVE_HIGH
    // ON=HIGH (bit置1), OFF=LOW (bit清0)
    if (on) pcf8574_output_state_ |=  (1 << pin);     // 置位: 或运算
    else    pcf8574_output_state_ &= ~(1 << pin);     // 清零: 与非运算
#else
    // ON=LOW (bit清0), OFF=HIGH (bit置1)
    if (on) pcf8574_output_state_ &= ~(1 << pin);
    else    pcf8574_output_state_ |=  (1 << pin);
#endif

    // 带重试的 I2C 写入: 避免单次通信失败导致继电器状态与内存不一致
    for (int retry = 0; retry < 3; retry++) {
        if (Pcf8574Write(pcf8574_output_state_)) return; // 写入成功, 立即返回
        esp_rom_delay_us(500);                           // 等待总线恢复后重试
    }
    // 3次重试均失败 — 继电器硬件状态可能与内存不一致!
    ESP_LOGW(TAG, "PCF8574写入失败(继电器%d=%s), state=0x%02X",
             relay_index, on ? "ON" : "OFF", pcf8574_output_state_);
}

/* ================================================================
 * 初始化传感器数据读取 (不含自动控制)
 * ----------------------------------------------------------------
 * 在系统启动时调用一次, 用于 LCD 显示初始值
 * DHT11 带 3 次重试, 每次间隔 1 秒 (符合 DHT11 最小采样间隔)
 * ================================================================ */
void PlantMonitor::ReadSensorsForInit() {
    // DHT11 读取 (最多重试 3 次, 间隔 1 秒)
    float temp = 0, hum = 0;
    for (int retry = 0; retry < 3; retry++) {
        if (Dht11Read(temp, hum)) {
            sensor_data_.temperature = temp;
            sensor_data_.humidity = hum;
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(1000));     // DHT11 要求采样间隔 ≥1秒
    }

    // 光照传感器单次读取
    int light = LightSensorRead();
    if (light >= 0) {
        sensor_data_.light_value = light;
    }

    // 土壤湿度: ADS1115 模拟量优先, PCF8574 数字量后备
    sensor_data_.soil_moisture_digital = (SoilMoistureReadDigital() == 1);
    if (ads1115_present_) {
        SoilMoistureReadAnalog();            // 读取模拟量 0-100%
    } else {
        // 无 ADS1115 时, 数字量映射: 1(潮湿)→100%, 0(干燥)→0%
        sensor_data_.soil_moisture_percent = sensor_data_.soil_moisture_digital ? 100 : 0;
    }

    ESP_LOGI(TAG, "初始传感器读数-温度:%.1f℃ 湿度:%.1f%% 光照:%d 土壤:%d%%(raw:%d dig:%d)",
             sensor_data_.temperature, sensor_data_.humidity,
             sensor_data_.light_value, sensor_data_.soil_moisture_percent,
             sensor_data_.soil_moisture_raw, sensor_data_.soil_moisture_digital);
}

/* ================================================================
 * 自动控制逻辑 — 回滞 (Hysteresis) 算法
 * ----------------------------------------------------------------
 * 每次 Update() 时调用, 根据当前传感器值与阈值比较, 控制继电器
 *
 * 回滞设计:
 *   开启和关闭使用不同阈值, 中间区域保持当前状态不变,
 *   避免在阈值边界频繁切换继电器 (继电器快速开关会损坏)
 *
 * 温度控制:  < temp_min → 开加热,  > temp_max → 关加热
 * 光照控制:  取决于 LIGHT_SENSOR_INVERTED 宏
 * 土壤控制:  < soil_moisture_dry → 开水泵,  ≥ dry+15 → 关水泵
 * ================================================================ */
void PlantMonitor::AutoControl() {
    // ---- 温度控制 ----
    if (sensor_data_.temperature < thresholds_.temp_min) {
        SetRelay(2, true);                      // 太冷 → 开加热片
    } else if (sensor_data_.temperature > thresholds_.temp_max) {
        SetRelay(2, false);                     // 太热 → 关加热片
    }

    // ---- 光照控制 ----
#ifdef LIGHT_SENSOR_INVERTED
    // 反转型传感器 (光照越强 ADC 越低):
    //   ADC > light_max → 太暗 → 开补光灯
    //   ADC ≤ light_max → 够亮 → 关补光灯
    if (sensor_data_.light_value > thresholds_.light_max && sensor_data_.light_value >= 0) {
        SetRelay(1, true);
    } else {
        SetRelay(1, false);
    }
#else
    // 标准传感器 (光照越强 ADC 越高):
    //   ADC < light_min → 太暗 → 开补光灯
    //   ADC ≥ light_min → 够亮 → 关补光灯
    if (sensor_data_.light_value < thresholds_.light_min && sensor_data_.light_value >= 0) {
        SetRelay(1, true);
    } else {
        SetRelay(1, false);
    }
#endif

    // ---- 土壤湿度 + 水泵控制 ----
    // 开启条件: 土壤湿度低于干燥阈值 或 空气湿度低于最低阈值
    bool need_water = (sensor_data_.soil_moisture_percent < thresholds_.soil_moisture_dry) ||
                      (sensor_data_.humidity < thresholds_.humidity_min);
    if (need_water) {
        SetRelay(0, true);                      // 缺水 → 开水泵
    } else if (sensor_data_.soil_moisture_percent >= thresholds_.soil_moisture_dry + 15) {
        // 回滞关闭: 土壤湿度回升到 dry+15% 以上才关水泵
        // 15% 的缓冲防止水泵在阈值附近频繁启停
        SetRelay(0, false);
    }
}

/* ================================================================
 * 自动 AI 模式 — 异常检测 + 声学回环触发小智播报
 * ----------------------------------------------------------------
 * 架构:
 *   每10秒巡检传感器 → 检测异常(越阈值) →
 *   触发声学回环: 打开小智对话通道 → 2.5秒后播放预录音频(喇叭)
 *   → 麦克风拾音 → 服务器STT识别 → AI调用MCP工具分析 → TTS播报
 *
 * 声学回环 (Acoustic Loopback):
 *   设备通过喇叭播放包含唤醒词+指令的预录音频,
 *   同时麦克风拾取该音频发送到服务器, 实现程序化触发AI。
 *
 * 冷却机制 (Cooldown):
 *   两次AI触发之间至少间隔60秒, 避免频繁播报
 * ================================================================ */

// 文件作用域静态变量: 跨函数共享冷却状态和延迟定时器
static bool auto_ai_cooldown_ = false;               // 60秒冷却标志
static esp_timer_handle_t announce_delay_timer_ = nullptr; // 2.5秒延迟播放定时器

/**
 * 触发小智播报 — 声学回环
 * 流程: 检查冷却 → 打开音频通道 → 延迟2.5秒 → 播放OGG → 启动60秒冷却
 */
static void TriggerXiaozhiAnnounce() {
    if (auto_ai_cooldown_) return;                   // 冷却期内, 跳过
    auto_ai_cooldown_ = true;

    auto& app = Application::GetInstance();
    if (app.GetDeviceState() != kDeviceStateIdle) {
        auto_ai_cooldown_ = false;                   // 小智正忙, 等下一轮
        return;
    }

    ESP_LOGI(TAG, "AutoAI: 触发小智播报");

    // 第1步: 在主线程打开音频通道 (ToggleChatState: idle → connecting → listening)
    app.Schedule([]() {
        Application::GetInstance().ToggleChatState();
    });

    // 第2步: 2.5秒后播放预录音频 (等待音频通道和麦克风就绪)
    if (announce_delay_timer_) {
        esp_timer_stop(announce_delay_timer_);
        esp_timer_delete(announce_delay_timer_);
    }
    esp_timer_create_args_t args = {
        .callback = [](void*) {
            auto& app = Application::GetInstance();
            if (app.GetDeviceState() != kDeviceStateListening) {
                auto_ai_cooldown_ = false;           // 未进入聆听状态, 放弃本次播报
                return;
            }

            // 验证 OGG 数据有效性 (至少包含完整 OGG 页头: 28字节)
            auto ogg = GetPlantAnnounceOgg();
            if (ogg.size() < 28) {
                ESP_LOGW(TAG, "AutoAI: OGG数据无效(%d字节), 请替换auto_announce.h中的占位数据", (int)ogg.size());
                auto_ai_cooldown_ = false;
                return;
            }

            ESP_LOGI(TAG, "AutoAI: 播放播报音频 (%d字节)", (int)ogg.size());
            // 在主线程播放 OGG (AudioService 非线程安全)
            std::string ogg_copy(ogg);
            app.Schedule([ogg_copy]() {
                Application::GetInstance().PlaySound(ogg_copy);
            });

            // 第3步: 启动 60 秒冷却定时器
            esp_timer_create_args_t cd_args = {
                .callback = [](void*) { auto_ai_cooldown_ = false; },
                .dispatch_method = ESP_TIMER_TASK,
                .name = "ai_cooldown",
                .skip_unhandled_events = false,
            };
            esp_timer_handle_t cd_timer;
            ESP_ERROR_CHECK(esp_timer_create(&cd_args, &cd_timer));
            ESP_ERROR_CHECK(esp_timer_start_once(cd_timer, 60000000)); // 60,000,000 μs = 60s
        },
        .dispatch_method = ESP_TIMER_TASK,
        .name = "announce_delay",
        .skip_unhandled_events = false,
    };
    ESP_ERROR_CHECK(esp_timer_create(&args, &announce_delay_timer_));
    ESP_ERROR_CHECK(esp_timer_start_once(announce_delay_timer_, 2500000)); // 2,500,000 μs = 2.5s
}

/**
 * 启动自动 AI 模式
 * 创建 10 秒周期定时器, 每次触发时检测所有传感器是否越过阈值
 */
void PlantMonitor::StartAutoAiMode() {
    if (auto_ai_enabled_) return;
    auto_ai_enabled_ = true;
    auto_ai_cooldown_ = false;                       // 重置冷却, 允许立即触发

    esp_timer_create_args_t args = {
        .callback = [](void* arg) {
            auto* self = static_cast<PlantMonitor*>(arg);
            self->Update();                          // 刷新传感器数据

            auto& s = self->sensor_data_;
            auto& t = self->thresholds_;
            bool abnormal = false;
            char reason[128] = "";

            // === 温度异常检测 (超过阈值区间) ===
            if (s.temperature > 0) {
                if (s.temperature < t.temp_min) {
                    snprintf(reason, sizeof(reason), "温度过低%.1f℃(下限%d℃)", s.temperature, t.temp_min);
                    abnormal = true;
                } else if (s.temperature > t.temp_max) {
                    snprintf(reason, sizeof(reason), "温度过高%.1f℃(上限%d℃)", s.temperature, t.temp_max);
                    abnormal = true;
                }
            }

            // === 湿度异常检测 ===
            if (!abnormal && s.humidity > 0) {
                if (s.humidity < t.humidity_min) {
                    snprintf(reason, sizeof(reason), "湿度过低%.0f%%(下限%d%%)", s.humidity, t.humidity_min);
                    abnormal = true;
                } else if (s.humidity > t.humidity_max) {
                    snprintf(reason, sizeof(reason), "湿度过高%.0f%%(上限%d%%)", s.humidity, t.humidity_max);
                    abnormal = true;
                }
            }

            // === 光照异常检测 (区分传感器类型) ===
            if (!abnormal && s.light_value >= 0) {
#ifdef LIGHT_SENSOR_INVERTED
                // 反转型: ADC高=暗, ADC低=亮
                if (s.light_value > t.light_max) {
                    snprintf(reason, sizeof(reason), "光照不足%d(>上限%d)", s.light_value, t.light_max);
                    abnormal = true;
                } else if (s.light_value < t.light_min) {
                    snprintf(reason, sizeof(reason), "光照过强%d(<下限%d)", s.light_value, t.light_min);
                    abnormal = true;
                }
#else
                if (s.light_value < t.light_min) {
                    snprintf(reason, sizeof(reason), "光照不足%d(<下限%d)", s.light_value, t.light_min);
                    abnormal = true;
                } else if (s.light_value > t.light_max) {
                    snprintf(reason, sizeof(reason), "光照过强%d(>上限%d)", s.light_value, t.light_max);
                    abnormal = true;
                }
#endif
            }

            // === 土壤湿度异常检测 ===
            if (!abnormal && s.soil_moisture_percent >= 0) {
                if (s.soil_moisture_percent < t.soil_moisture_dry) {
                    snprintf(reason, sizeof(reason), "土壤干燥%d%%(浇水阈值%d%%)",
                             s.soil_moisture_percent, t.soil_moisture_dry);
                    abnormal = true;
                }
            }

            // 任一传感器异常 → 触发声学回环, 唤醒小智AI
            if (abnormal) {
                ESP_LOGI(TAG, "AutoAI: 检测到异常 - %s", reason);
                TriggerXiaozhiAnnounce();
            }
        },
        .arg = this,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "auto_ai_timer",
        .skip_unhandled_events = false,
    };
    ESP_ERROR_CHECK(esp_timer_create(&args, &auto_ai_timer_));
    // AUTO_AI_INTERVAL_MS 毫秒 → 微秒 (*1000)
    ESP_ERROR_CHECK(esp_timer_start_periodic(auto_ai_timer_, AUTO_AI_INTERVAL_MS * 1000));
    ESP_LOGI(TAG, "自动AI模式已启动 (间隔%d秒, 异常时触发小智播报)", AUTO_AI_INTERVAL_MS / 1000);
}

/** 停止自动 AI 模式, 释放所有相关定时器 */
void PlantMonitor::StopAutoAiMode() {
    if (!auto_ai_enabled_) return;
    auto_ai_enabled_ = false;
    if (auto_ai_timer_) {
        esp_timer_stop(auto_ai_timer_);
        esp_timer_delete(auto_ai_timer_);
        auto_ai_timer_ = nullptr;
    }
    if (announce_delay_timer_) {
        esp_timer_stop(announce_delay_timer_);
        esp_timer_delete(announce_delay_timer_);
        announce_delay_timer_ = nullptr;
    }
    ESP_LOGI(TAG, "自动AI模式已停止");
}

/* ================================================================
 * 传感器数据周期性更新
 * ----------------------------------------------------------------
 * 由 5 秒定时器调用 (Init 后延迟 3 秒启动)
 * 读取 DHT11 (3次重试, 1秒间隔) + 光照 ADC + 土壤湿度 → 执行自动控制
 * ================================================================ */
void PlantMonitor::Update() {
    if (!initialized_) return;

    // 读取 DHT11 (最多重试 3 次, 每次间隔 1 秒, 符合 DHT11 采样规范)
    float temp = 0, hum = 0;
    for (int retry = 0; retry < 3; retry++) {
        if (Dht11Read(temp, hum)) {
            sensor_data_.temperature = temp;
            sensor_data_.humidity = hum;
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(1000));         // DHT11 要求 ≥1秒 采样间隔
    }

    // 读取光照传感器
    int light = LightSensorRead();
    if (light >= 0) {
        sensor_data_.light_value = light;
    }

    // 读取土壤湿度 (ADS1115 模拟量优先, PCF8574 数字量后备)
    sensor_data_.soil_moisture_digital = (SoilMoistureReadDigital() == 1);
    if (ads1115_present_) {
        SoilMoistureReadAnalog();
    } else {
        sensor_data_.soil_moisture_percent = sensor_data_.soil_moisture_digital ? 100 : 0;
    }

    // 执行自动控制逻辑 (继电器开关)
    AutoControl();

    ESP_LOGI(TAG, "温度:%.1f℃ 湿度:%.1f%% 光照:%d 土壤:%d%% 水泵:%d 灯光:%d 加热:%d",
             sensor_data_.temperature, sensor_data_.humidity,
             sensor_data_.light_value, sensor_data_.soil_moisture_percent,
             sensor_data_.relay_pump, sensor_data_.relay_light, sensor_data_.relay_heater);
}

/* ================================================================
 * 阈值设置 (供 AI MCP 工具和 OneNET 远程命令调用)
 * ================================================================ */
void PlantMonitor::SetThresholds(const SensorThresholds& t) {
    thresholds_ = t;                               // 整体结构体拷贝 (POD, 安全)
    ESP_LOGI(TAG, "阈值已更新: T[%d-%d]℃ H[%d-%d]%% L[%d-%d] S[%d%%]",
             thresholds_.temp_min, thresholds_.temp_max,
             thresholds_.humidity_min, thresholds_.humidity_max,
             thresholds_.light_min, thresholds_.light_max,
             thresholds_.soil_moisture_dry);
}

/* ================================================================
 * MCP 工具注册 — 11个 AI 可调用工具
 * ----------------------------------------------------------------
 * MCP (Model Context Protocol) 是设备与 AI 之间的 JSON-RPC 协议
 * 小智 AI 服务器可以通过 MCP 调用这些工具来读取传感器、调整阈值、控制继电器
 *
 * 工具列表:
 *   plant.get_sensor_data              获取传感器数据
 *   plant.set_temp_threshold            设置温度阈值
 *   plant.set_humidity_threshold        设置湿度阈值
 *   plant.set_light_threshold           设置光照阈值
 *   plant.set_soil_moisture_threshold   设置土壤湿度阈值
 *   plant.get_thresholds                获取所有阈值
 *   plant.control_pump                  手动控制水泵
 *   plant.control_light                 手动控制补光灯
 *   plant.control_heater                手动控制加热片
 *   plant.adjust_all_thresholds         一次性调整所有阈值
 *   plant.analyze_environment           综合分析 (含摄像头 + 花卉知识库)
 * ================================================================ */
void PlantMonitor::RegisterMcpTools() {
    auto& mcp = McpServer::GetInstance();

    // ---- 工具 1: 获取传感器数据 ----
    mcp.AddTool(
        "plant.get_sensor_data",
        "获取植物生长环境的传感器数据。返回温度、湿度、光照、土壤湿度等当前值。\n"
        "当用户询问植物生长状态、环境参数时使用此工具。\n"
        "花卉养护参考：多数开花植物最适温度15-30℃、湿度40-80%；多肉植物温度10-35℃、湿度30-50%；热带植物温度20-35℃、湿度60-90%。",
        PropertyList(),
        [this](const PropertyList&) -> ReturnValue {
            Update();                              // 先刷新传感器数据
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

    // ---- 工具 2: 设置温度阈值 ----
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

    // ---- 工具 3: 设置湿度阈值 ----
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

    // ---- 工具 4: 设置光照阈值 ----
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

    // ---- 工具 5: 获取当前阈值 ----
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

    // ---- 工具 6: 设置土壤湿度阈值 ----
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

    // ---- 工具 7: 手动控制水泵 ----
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

    // ---- 工具 8: 手动控制补光灯 ----
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

    // ---- 工具 9: 手动控制加热片 ----
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

    // ---- 工具 10: AI 综合分析 — 一次性调整所有阈值 ----
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

    // ---- 工具 11: 植物环境综合分析 (含摄像头 + 花卉养护知识库) ----
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
            Update();                              // 刷新传感器数据
            auto& s = sensor_data_;

            // 捕获摄像头帧 (供后续 AI 图像分析使用)
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

/* ================================================================
 * 系统初始化 — 7 步启动流程
 * ----------------------------------------------------------------
 * 必须在摄像头初始化完成后调用 (I2C_NUM_0 已由 SCCB 配置)
 *
 * 步骤:
 *   1. PCF8574 I2C GPIO 扩展器 (继电器控制 + 土壤数字量)
 *   2. ADS1115 I2C ADC (土壤模拟量)
 *   3. 光照传感器 ADC (ESP32-S3 ADC1)
 *   4. DHT11 GPIO 单总线
 *   5. 注册 11 个 MCP 工具
 *   6. 初始传感器读取 (用于 LCD 显示)
 *   7. 3秒后启动 5秒周期自动控制定时器
 * ================================================================ */
void PlantMonitor::Initialize() {
    if (initialized_) {
        ESP_LOGW(TAG, "PlantMonitor已经初始化");
        return;
    }
    ESP_LOGI(TAG, "正在初始化植物监控系统...");

    // 1. 初始化 PCF8574 (依赖摄像头 SCCB 已配置的 I2C_NUM_0 总线)
    if (!Pcf8574Init()) {
        ESP_LOGW(TAG, "PCF8574初始化失败，继电器功能不可用");
    }

    // 2. 初始化 ADS1115 (与 PCF8574 共用 I2C 总线, 在 PCF8574 之后)
    ads1115_present_ = Ads1115Init();

    // 3. 初始化光照传感器 ADC (ESP32-S3 内置 ADC1, 12-bit, 0-3.3V)
    adc_oneshot_unit_init_cfg_t adc_cfg = {
        .unit_id = LIGHT_SENSOR_ADC_UNIT,        // ADC_UNIT_1 (不受 WiFi 干扰)
        .clk_src = ADC_RTC_CLK_SRC_DEFAULT,      // 使用 RTC 时钟源, 低噪声
        .ulp_mode = ADC_ULP_MODE_DISABLE,        // 禁用超低功耗模式
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&adc_cfg, &adc_handle_));

    adc_oneshot_chan_cfg_t chan_cfg = {
        .atten = ADC_ATTEN_DB_12,                // 12dB衰减 → 0-3.3V 量程
        .bitwidth = ADC_BITWIDTH_12,             // 12-bit → 0-4095
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle_, LIGHT_SENSOR_ADC_CHANNEL, &chan_cfg));

    // 4. 初始化 DHT11 GPIO (开漏输出 + 内部上拉 = 单总线模式)
    Dht11PinInit(DHT11_DATA_PIN);

    // 5. 注册 11 个 MCP 工具 (AI 可调用的传感器/控制接口)
    RegisterMcpTools();

    // 6. 读取一次传感器数据 (用于 LCD 初始显示, 不执行自动控制)
    ReadSensorsForInit();

    // 7. 创建 3 秒延迟定时器, 到期后启动 5 秒周期自动控制
    //    延迟 3 秒是为了避免启动时传感器读数不稳定导致错误控制
    esp_timer_create_args_t init_delay_args = {
        .callback = [](void* arg) {
            PlantMonitor* self = static_cast<PlantMonitor*>(arg);
            // 3秒延迟到期: 创建真正的 5秒周期定时器
            esp_timer_create_args_t timer_args = {
                .callback = [](void* timer_arg) {
                    PlantMonitor* monitor = static_cast<PlantMonitor*>(timer_arg);
                    monitor->Update();           // 每5秒: 读传感器 → 自动控制
                },
                .arg = self,
                .dispatch_method = ESP_TIMER_TASK,
                .name = "plant_monitor_timer",
                .skip_unhandled_events = false,
            };
            ESP_ERROR_CHECK(esp_timer_create(&timer_args, &self->update_timer_));
            ESP_ERROR_CHECK(esp_timer_start_periodic(self->update_timer_, 5000000)); // 5,000,000 μs = 5s
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
    ESP_ERROR_CHECK(esp_timer_start_once(init_delay_timer_, 3000000)); // 3,000,000 μs = 3s

    initialized_ = true;
    ESP_LOGI(TAG, "植物监控系统初始化完成");
}
