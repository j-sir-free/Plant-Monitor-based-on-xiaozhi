/**
 * @file    plant_monitor.h
 * @brief   植物生长环境监控系统 — 数据结构与核心类声明
 * @author  JJD-YOURFATHER
 *
 * PlantMonitor 是植物监控系统的核心单例类, 负责:
 *   - 传感器驱动 (DHT11, 光照ADC, PCF8574 I2C, ADS1115 ADC)
 *   - 继电器控制 (水泵/补光灯/加热片)
 *   - 自动控制逻辑 (根据阈值自动开关继电器)
 *   - 自动AI模式 (异常检测 + 声学回环触发小智AI)
 *   - MCP 工具注册 (11个AI可调用的传感器/控制工具)
 *
 * 数据流:
 *   硬件传感器 → SensorData 结构体 → LCD显示 / Web API / OneNET云 / MCP AI
 *   AI阈值调整 → SensorThresholds → AutoControl → 继电器 → 执行器
 */
#ifndef PLANT_MONITOR_H
#define PLANT_MONITOR_H

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/event_groups.h>
#include <driver/gpio.h>
#include <esp_adc/adc_oneshot.h>
#include <esp_timer.h>

#include "mcp_server.h"
#include "config.h"

/* ================================================================
 * SensorThresholds — 传感器控制阈值 (由AI动态调整)
 * ----------------------------------------------------------------
 * 温度: [temp_min, temp_max] — 理想温度区间 (℃)
 * 湿度: [humidity_min, humidity_max] — 理想湿度区间 (%)
 * 光照: [light_min, light_max] — 理想光照 ADC 区间
 *   - 正常传感器: light_min=暗(开灯阈值), light_max=亮(关灯阈值)
 *   - 反转型传感器: light_min=亮(关灯阈值), light_max=暗(开灯阈值)
 * 土壤: soil_moisture_dry — 干燥阈值 (%), 低于此值开水泵
 * ================================================================ */
struct SensorThresholds {
    int temp_min = 18;              // 最低理想温度 (℃)
    int temp_max = 32;              // 最高理想温度 (℃)
    int humidity_min = 20;          // 最低理想湿度 (%)
    int humidity_max = 90;          // 最高理想湿度 (%)
    int light_min = 1000;           // 光照下限 (反转型=亮/关灯, 标准型=暗/开灯)
    int light_max = 1500;           // 光照上限 (反转型=暗/开灯, 标准型=亮/关灯)
    int soil_moisture_dry = 20;     // 土壤干燥阈值 (%, 低于此值开水泵)
};

/* ================================================================
 * SensorData — 传感器实时数据快照
 * ----------------------------------------------------------------
 * 在每次 Update() 调用时刷新, 由 AutoControl 和显示系统消费
 * ================================================================ */
struct SensorData {
    float temperature = 0.0f;               // DHT11 温度 (℃), 精度 ±1℃
    float humidity = 0.0f;                  // DHT11 湿度 (%), 精度 ±1%
    int light_value = 0;                    // 光照 ADC 原始值 (0-4095, 12-bit)
    int soil_moisture_raw = 0;              // 土壤湿度 ADS1115 16-bit ADC 原始值
    int soil_moisture_percent = 0;          // 土壤湿度 百分比 (0-100%), 线性映射
    bool soil_moisture_digital = false;     // 土壤湿度 数字量 (PCF8574 DO, ADS1115 未检测时的后备)
    bool relay_pump = false;                // 水泵继电器状态 (true=通电运行)
    bool relay_light = false;               // 补光灯继电器状态 (true=通电亮灯)
    bool relay_heater = false;              // 加热片继电器状态 (true=通电加热)
};

/**
 * PlantMonitor — 植物监控系统核心单例
 *
 * 使用 Meyer's 单例模式 (C++11 线程安全静态局部变量)。
 * 初始化顺序: PCF8574 → ADS1115 → ADC → DHT11 → MCP → 初始读数 → 定时自动控制
 */
class PlantMonitor {
public:
    /** Meyer's 单例 — 线程安全的懒汉式初始化 */
    static PlantMonitor& GetInstance() {
        static PlantMonitor instance;
        return instance;
    }

    /**
     * 初始化所有传感器和外设
     * 应在摄像头初始化完成后调用 (I2C_NUM_0 已由 SCCB 配置)
     *
     * 初始化流程 (7步):
     *   1. PCF8574 I2C GPIO 扩展器
     *   2. ADS1115 I2C ADC (PCF8574 之后, 共用总线)
     *   3. 光照传感器 ADC (GPIO3=ADC1_CH2)
     *   4. DHT11 GPIO (开漏+上拉=单总线)
     *   5. 注册 11 个 MCP 工具
     *   6. 读取一次传感器数据 (用于 LCD 初始显示)
     *   7. 3秒后启动 5秒周期自动控制定时器
     */
    void Initialize();

    /** 获取最新传感器数据快照 (线程安全, 原子拷贝) */
    SensorData GetSensorData() { return sensor_data_; }

    /** 获取当前阈值配置 (线程安全, 原子拷贝) */
    SensorThresholds GetThresholds() { return thresholds_; }

    /** 设置新阈值 (供 AI MCP 工具或 OneNET 远程命令调用) */
    void SetThresholds(const SensorThresholds& t);

    /** 执行一次传感器读取 + 自动控制判断 (由 5秒 定时器调用) */
    void Update();

    /**
     * 手动控制继电器
     * @param relay_index  0=水泵, 1=补光灯, 2=加热片
     * @param on           true=开启, false=关闭
     *
     * RELAY_ACTIVE_HIGH 模式下:
     *   ON  → PCF8574输出HIGH → 继电器不吸合 → COM-NC连通 → 设备通电
     *   OFF → PCF8574输出LOW  → 继电器吸合   → COM-NO连通 → 设备断电
     */
    void SetRelay(int relay_index, bool on);

    // ---- 自动 AI 模式 ----

    /**
     * 启动自动AI模式 — 每10秒检测传感器异常
     * 当温度/湿度/光照/土壤越过阈值时, 通过声学回环触发小智AI播报
     * 两次触发之间至少间隔60秒冷却期
     */
    void StartAutoAiMode();

    /** 停止自动AI模式 */
    void StopAutoAiMode();

    /** 查询自动AI模式是否开启 */
    bool IsAutoAiMode() const { return auto_ai_enabled_; }

private:
    PlantMonitor();
    ~PlantMonitor();
    PlantMonitor(const PlantMonitor&) = delete;              // 禁止拷贝
    PlantMonitor& operator=(const PlantMonitor&) = delete;   // 禁止赋值

    // ==================== 传感器驱动层 ====================

    /**
     * DHT11 单总线读取
     * 协议: 主机拉低≥18ms → 拉高30μs → DHT11应答(低80μs+高80μs) → 5字节数据
     * 数据格式: [湿度整数][湿度小数][温度整数][温度小数][校验和]
     * @return true=读取成功且校验通过
     */
    bool Dht11Read(float& temperature, float& humidity);

    /**
     * 光照传感器 ADC 读取 (ESP32-S3 内置 ADC1)
     * GPIO3=ADC1_CH2, 12-bit, 12dB衰减 (0-3.3V)
     * @return ADC原始值 (0-4095), -1=读取失败
     */
    int LightSensorRead();

    // ==================== PCF8574 I2C 位模拟驱动层 ====================
    // 使用 GPIO4(SDA) + GPIO5(SCL) 手动模拟 I2C 时序
    // 与摄像头 SCCB 共享引脚但独立操作, 不依赖 ESP-IDF I2C 驱动
    // 时序: 5μs 延迟 → ~100kHz 时钟

    /** PCF8574 初始化: 探测芯片 + 设置所有输出为 HIGH */
    bool Pcf8574Init();

    /** 向 PCF8574 写入 8-bit GPIO 状态, @return true=收到ACK */
    bool Pcf8574Write(uint8_t data);

    /** 读取 PCF8574 8-bit GPIO 输入状态 */
    uint8_t Pcf8574Read();

    // ==================== ADS1115 I2C ADC 驱动层 ====================
    // 16-bit 模数转换器, I2C 地址 0x48, 与 PCF8574 共用总线

    /** ADS1115 初始化: 写配置寄存器 + 验证读取 */
    bool Ads1115Init();

    /**
     * 读取 ADS1115 16-bit 转换结果
     * 2步I2C操作: 写寄存器指针0x00 → 读2字节 (MSB first)
     * @return true=读取成功
     */
    bool Ads1115ReadConversion(int16_t* result);

    // ==================== 传感器数据采集 ====================

    /** 读取土壤湿度模拟量 (ADS1115 AIN0), 并线性映射到 0-100% */
    void SoilMoistureReadAnalog();

    /** 读取土壤湿度数字量 (PCF8574 P4 DO 引脚), 返回 0/1 */
    int SoilMoistureReadDigital();

    /** 仅读取传感器数据 (不执行自动控制), 用于初始化时 LCD 显示 */
    void ReadSensorsForInit();

    // ==================== 控制逻辑 ====================

    /**
     * 自动控制逻辑 (每次 Update 时执行)
     * 温度: <temp_min→开加热, >temp_max→关加热
     * 光照: 根据传感器类型判断暗/亮, 开关补光灯
     * 土壤: <soil_moisture_dry→开水泵, ≥dry+15→关水泵
     *
     * 回滞 (Hysteresis) 设计:
     *   开和关使用不同的阈值, 中间区域保持当前状态,
     *   避免在阈值边界频繁开关继电器
     */
    void AutoControl();

    /** 注册 11 个 MCP 工具, 供小智 AI 通过 JSON-RPC 调用 */
    void RegisterMcpTools();

    // ==================== 成员变量 ====================

    SensorData sensor_data_;                            // 传感器数据快照
    SensorThresholds thresholds_;                       // 控制阈值 (可由AI修改)

    adc_oneshot_unit_handle_t adc_handle_;              // 光照传感器 ADC 句柄
    esp_timer_handle_t update_timer_;                   // 5秒周期更新定时器
    esp_timer_handle_t init_delay_timer_ = nullptr;     // 3秒延迟启动定时器

    bool initialized_ = false;                          // 初始化完成标志
    uint8_t pcf8574_output_state_;                      // PCF8574 当前输出状态 (bitmask)
    bool ads1115_present_ = false;                      // ADS1115 是否检测到
    bool auto_control_enabled_ = false;                 // 自动控制是否已启动
    bool auto_ai_enabled_ = false;                      // 自动AI模式是否已开启
    esp_timer_handle_t auto_ai_timer_ = nullptr;        // 自动AI 10秒巡检定时器
};

/** 便捷访问单例的辅助函数 */
inline PlantMonitor& GetPlantMonitor() {
    return PlantMonitor::GetInstance();
}

#endif // PLANT_MONITOR_H
