#ifndef ONENET_CLIENT_H
#define ONENET_CLIENT_H

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/event_groups.h>
#include <esp_timer.h>
#include <esp_event.h>
#include <string>
#include <functional>

struct SensorData;

// OneNET 下发命令回调类型
// 参数: JSON格式的命令字符串
// 返回: 处理结果的JSON字符串
using OnenetCommandCallback = std::function<std::string(const std::string& cmd_json)>;

class OnenetClient {
public:
    static OnenetClient& GetInstance() {
        static OnenetClient instance;
        return instance;
    }

    // 初始化并启动OneNET连接
    bool Initialize();

    // 设置收到云平台命令时的回调
    void SetCommandCallback(OnenetCommandCallback callback) {
        command_callback_ = callback;
    }

    // 上传传感器数据到OneNET云平台
    bool UploadSensorData(const SensorData& data);

    // 是否已连接到OneNET
    bool IsConnected() const { return connected_; }

    // 上报设备状态（继电器等）
    bool ReportDeviceStatus(const SensorData& data);

private:
    OnenetClient();
    ~OnenetClient();
    OnenetClient(const OnenetClient&) = delete;
    OnenetClient& operator=(const OnenetClient&) = delete;

    // MQTT事件处理回调
    static void MqttEventHandler(void* handler_args, esp_event_base_t base,
                                 int32_t event_id, void* event_data);

    // 处理接收到的MQTT消息
    void HandleMqttData(const std::string& topic, const std::string& data);

    // 订阅OneNET命令主题
    void SubscribeTopics();

    // 连接OneNET MQTT
    void Connect();

    // 定时上报任务
    static void UploadTimerCallback(void* arg);

    void* mqtt_client_;           // esp_mqtt_client_handle_t
    esp_timer_handle_t upload_timer_;
    OnenetCommandCallback command_callback_;
    bool connected_;
    bool config_ok_;              // 配置是否就绪
    bool initialized_;
    int msg_id_;
};

inline OnenetClient& GetOnenetClient() {
    return OnenetClient::GetInstance();
}

#endif // ONENET_CLIENT_H
