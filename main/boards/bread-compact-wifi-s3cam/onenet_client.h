/**
 * @file    onenet_client.h
 * @brief   OneNET 云平台 MQTT 客户端 — 传感器数据上传与远程控制命令接收
 * @author  JJD-YOURFATHER
 *
 * 基于 ESP-MQTT 组件实现, 单例模式。
 * 使用 OneNET v5.0 MQTT 协议 (非 TLS)。
 *
 * 数据流:
 *   上传 — 定时器→读取PlantMonitor→JSON序列化→MQTT Publish→thing/property/post
 *   接收 — MQTT订阅→thing/property/set→JSON解析→回调→阈值调整/继电器控制
 */
#ifndef ONENET_CLIENT_H
#define ONENET_CLIENT_H

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/event_groups.h>
#include <esp_timer.h>
#include <esp_event.h>
#include <string>
#include <functional>

struct SensorData;  // 前向声明, 避免循环依赖 (定义在 plant_monitor.h)

/**
 * OneNET 云平台下发命令回调
 * @param  cmd_json  JSON 格式命令 (如: {"temp_min":18,"temp_max":32})
 * @return JSON 格式的处理结果
 */
using OnenetCommandCallback = std::function<std::string(const std::string& cmd_json)>;

class OnenetClient {
public:
    /** Meyer's 单例 — 线程安全的懒汉式初始化 */
    static OnenetClient& GetInstance() {
        static OnenetClient instance;
        return instance;
    }

    /**
     * 初始化 MQTT 客户端
     * 配置 MQTT 连接参数, 启动15秒周期上传定时器
     * 如果 OneNET 未配置 (ONENET_PRODUCT_ID 为空), 跳过初始化
     * @return true=初始化成功, false=未配置
     */
    bool Initialize();

    /** 注册云平台命令回调, 在 compact_wifi_board_s3cam.cc 中设置 */
    void SetCommandCallback(OnenetCommandCallback callback) {
        command_callback_ = callback;
    }

    /**
     * 上传传感器数据 (JSON格式) 到 OneNET
     * 发布到 $sys/{pid}/{device}/thing/property/post
     * @return true=发布成功, false=未连接或发布失败
     */
    bool UploadSensorData(const SensorData& data);

    /** 查询 MQTT 连接状态 */
    bool IsConnected() const { return connected_; }

    /**
     * 上报设备事件 (继电器状态变更)
     * 发布到 $sys/{pid}/{device}/thing/event/post
     */
    bool ReportDeviceStatus(const SensorData& data);

private:
    OnenetClient();
    ~OnenetClient();
    OnenetClient(const OnenetClient&) = delete;              // 禁止拷贝
    OnenetClient& operator=(const OnenetClient&) = delete;   // 禁止赋值

    /** MQTT 事件处理器 — 处理连接/断开/数据到达/错误 */
    static void MqttEventHandler(void* handler_args, esp_event_base_t base,
                                 int32_t event_id, void* event_data);

    /** 解析接收到的 MQTT 消息, 提取 JSON 命令并回调 */
    void HandleMqttData(const std::string& topic, const std::string& data);

    /** 订阅 OneNET 命令主题: thing/property/set */
    void SubscribeTopics();

    /** 建立 MQTT TCP 连接 */
    void Connect();

    /** 定时上传回调 (15秒周期) */
    static void UploadTimerCallback(void* arg);

    void* mqtt_client_;                         // esp_mqtt_client_handle_t (不透明指针)
    esp_timer_handle_t upload_timer_;           // 定时上传定时器
    OnenetCommandCallback command_callback_;    // 命令回调函数
    bool connected_;                            // MQTT 连接状态
    bool config_ok_;                            // 配置是否有效
    bool initialized_;                          // 是否已完成初始化
    int msg_id_;                                // MQTT 消息ID (递增)
};

/** 便捷访问单例的辅助函数 */
inline OnenetClient& GetOnenetClient() {
    return OnenetClient::GetInstance();
}

#endif // ONENET_CLIENT_H
