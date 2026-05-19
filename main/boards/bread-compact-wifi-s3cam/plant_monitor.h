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

// 传感器数据阈值结构体（由AI动态调整）
struct SensorThresholds {
    int temp_min = 18;          // 最低温度 (℃)
    int temp_max = 32;          // 最高温度 (℃)
    int humidity_min = 40;      // 最低湿度 (%)
    int humidity_max = 85;      // 最高湿度 (%)
    int light_min = 2000;        // 最低光照 (ADC原始值)
    int light_max = 3000;       // 最高光照 (ADC原始值)
    int soil_moisture_dry = 0;  // 土壤干燥阈值（DO=0表示干燥）
};

// 传感器数据结构体
struct SensorData {
    float temperature = 0.0f;   // 温度 (℃)
    float humidity = 0.0f;      // 湿度 (%)
    int light_value = 0;        // 光照 ADC 值 (0-4095)
    int soil_moisture = 0;      // 土壤湿度 (0=干燥, 1=潮湿)
    bool relay_pump = false;    // 水泵状态
    bool relay_light = false;   // 补光灯状态
    bool relay_heater = false;  // 加热片状态
};

class PlantMonitor {
public:
    static PlantMonitor& GetInstance() {
        static PlantMonitor instance;
        return instance;
    }

    // 初始化所有传感器和外设（在camera初始化之后调用）
    void Initialize();

    // 获取最新传感器数据
    SensorData GetSensorData() { return sensor_data_; }

    // 获取当前阈值配置
    SensorThresholds GetThresholds() { return thresholds_; }

    // 设置新的阈值（供AI MCP工具调用）
    void SetThresholds(const SensorThresholds& t);

    // 执行一次传感器读取和控制判断
    void Update();

    // 手动控制继电器
    void SetRelay(int relay_index, bool on);

private:
    PlantMonitor();
    ~PlantMonitor();
    PlantMonitor(const PlantMonitor&) = delete;
    PlantMonitor& operator=(const PlantMonitor&) = delete;

    // DHT11 驱动
    bool Dht11Read(float& temperature, float& humidity);

    // 光照传感器 ADC 读取
    int LightSensorRead();

    // PCF8574 I2C 驱动
    bool Pcf8574Init();
    bool Pcf8574Write(uint8_t data);
    uint8_t Pcf8574Read();

    // 土壤湿度读取（通过PCF8574 DO引脚）
    int SoilMoistureRead();

    // 自动控制逻辑：根据传感器值和阈值控制继电器
    void AutoControl();

    // 注册MCP工具供小智AI调用
    void RegisterMcpTools();

    // 成员变量
    SensorData sensor_data_;
    SensorThresholds thresholds_;
    adc_oneshot_unit_handle_t adc_handle_;
    esp_timer_handle_t update_timer_;
    bool initialized_ = false;
    uint8_t pcf8574_output_state_;  // PCF8574当前输出状态
};

// 单例访问辅助函数
inline PlantMonitor& GetPlantMonitor() {
    return PlantMonitor::GetInstance();
}

#endif // PLANT_MONITOR_H
