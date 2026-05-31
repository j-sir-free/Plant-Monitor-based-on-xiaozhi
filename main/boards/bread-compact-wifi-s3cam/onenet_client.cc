/**
 * @file    onenet_client.cc
 * @brief   OneNET 云平台 MQTT 客户端实现 — 传感器数据上传与远程命令处理
 * @author  JJD-YOURFATHER
 *
 * OneNET 是中国移动物联网开放平台 (v5.0 MQTT 协议)
 *
 * 功能:
 *   - 15秒周期传感器数据上传
 *   - 实时接收云平台下发命令 (阈值设置/继电器控制/数据刷新)
 *   - MQTT 主题订阅与响应 (thing/property/set → set_reply, cmd/request → cmd/response)
 *   - 自动重连 (周期性检查连接状态, 断开后自动重连)
 *
 * MQTT 主题:
 *   上传:  $sys/{pid}/{device}/thing/property/post   (传感器数据)
 *   事件:  $sys/{pid}/{device}/thing/event/post      (继电器状态)
 *   接收:  $sys/{pid}/{device}/thing/property/set    (云平台命令)
 *   响应:  $sys/{pid}/{device}/thing/property/set_reply
 */
#include "onenet_client.h"
#include "plant_monitor.h"
#include "config.h"

#include <esp_log.h>
#include <esp_timer.h>
#include <mqtt_client.h>
#include <cJSON.h>

#define TAG "OneNET"

OnenetClient::OnenetClient() {
    mqtt_client_ = nullptr;
    upload_timer_ = nullptr;
    connected_ = false;
    config_ok_ = false;
    initialized_ = false;
    msg_id_ = 0;                                 // MQTT 消息 ID 计数器
}

OnenetClient::~OnenetClient() {
    if (upload_timer_) {
        esp_timer_stop(upload_timer_);
        esp_timer_delete(upload_timer_);
    }
    if (mqtt_client_) {
        esp_mqtt_client_stop((esp_mqtt_client_handle_t)mqtt_client_);
        esp_mqtt_client_destroy((esp_mqtt_client_handle_t)mqtt_client_);
    }
}

/**
 * MQTT 事件处理器 (ESP-MQTT 回调)
 * MQTT_EVENT_CONNECTED:    订阅命令主题
 * MQTT_EVENT_DISCONNECTED: 标记断开连接
 * MQTT_EVENT_DATA:         解析接收到的消息
 * MQTT_EVENT_ERROR:        标记断开, 等待重连
 */
void OnenetClient::MqttEventHandler(void* handler_args, esp_event_base_t base,
                                    int32_t event_id, void* event_data) {
    auto* self = static_cast<OnenetClient*>(handler_args);
    auto* event = static_cast<esp_mqtt_event_handle_t>(event_data);

    switch ((esp_mqtt_event_id_t)event_id) {
        case MQTT_EVENT_CONNECTED:
            ESP_LOGI(TAG, "OneNET MQTT已连接");
            self->connected_ = true;
            self->SubscribeTopics();             // 订阅 thing/property/set
            break;

        case MQTT_EVENT_DISCONNECTED:
            ESP_LOGW(TAG, "OneNET MQTT断开连接");
            self->connected_ = false;
            break;

        case MQTT_EVENT_DATA: {
            // 提取 topic 和 payload (从 mqtt_event 中)
            std::string topic(event->topic, event->topic_len);
            std::string data(event->data, event->data_len);
            ESP_LOGI(TAG, "收到命令: topic=%.*s", event->topic_len, event->topic);
            self->HandleMqttData(topic, data);
            break;
        }

        case MQTT_EVENT_ERROR:
            ESP_LOGE(TAG, "OneNET MQTT错误");
            self->connected_ = false;
            break;

        default:
            break;
    }
}

/**
 * 处理接收到的 MQTT 数据
 *
 * 消息结构 (OneNET v5.0):
 *   {"id":"xxx","version":"1.0","params":{...}}
 *
 * 处理逻辑:
 *   1. 提取 params 字段作为命令 JSON
 *   2. 调用 command_callback_ 执行命令
 *   3. 如果是 cmd/request → 回复 cmd/response
 *   4. 如果是 thing/property/set → 回复 set_reply (ACK)
 */
void OnenetClient::HandleMqttData(const std::string& topic, const std::string& data) {
    if (!command_callback_) return;

    cJSON* root = cJSON_Parse(data.c_str());
    if (!root) return;

    // OneNET 消息格式: 顶层 JSON 包含 params 字段
    cJSON* params = cJSON_GetObjectItem(root, "params");
    std::string cmd_json;
    if (params) {
        char* str = cJSON_PrintUnformatted(params);   // 提取 params 为独立 JSON 字符串
        cmd_json = str;
        cJSON_free(str);
    } else {
        cmd_json = data;                              // 无 params, 直接用原始消息
    }

    // 执行命令回调 (定义在 compact_wifi_board_s3cam.cc)
    std::string response = command_callback_(cmd_json);

    // ---- 回复 cmd/request (OneNET 服务调用) ----
    const char* cmdid_prefix = "/cmd/request/";
    size_t pos = topic.find(cmdid_prefix);
    if (pos != std::string::npos) {
        std::string cmdid = topic.substr(pos + strlen(cmdid_prefix));
        char reply_topic[256];
        snprintf(reply_topic, sizeof(reply_topic),
            "$sys/%s/%s/cmd/response/%s",
            ONENET_PRODUCT_ID, ONENET_DEVICE_NAME, cmdid.c_str());
        esp_mqtt_client_publish((esp_mqtt_client_handle_t)mqtt_client_,
            reply_topic, response.c_str(), response.length(), 0, 0);
    }

    // ---- 回复 thing/property/set (属性设置确认) ----
    if (topic.find("thing/property/set") != std::string::npos) {
        cJSON* id_json = cJSON_GetObjectItem(root, "id");
        std::string msg_id = id_json && cJSON_IsString(id_json) ? id_json->valuestring : "0";
        char reply_topic[256];
        snprintf(reply_topic, sizeof(reply_topic),
            "$sys/%s/%s/thing/property/set_reply",
            ONENET_PRODUCT_ID, ONENET_DEVICE_NAME);
        // 标准 ACK: {"id":"...","code":200,"data":{}}
        char reply_payload[128];
        snprintf(reply_payload, sizeof(reply_payload),
            "{\"id\":\"%s\",\"code\":200,\"data\":{}}", msg_id.c_str());
        esp_mqtt_client_publish((esp_mqtt_client_handle_t)mqtt_client_,
            reply_topic, reply_payload, strlen(reply_payload), 0, 0);
    }

    cJSON_Delete(root);
}

/**
 * 订阅 OneNET 命令主题
 * 主题: $sys/{产品ID}/{设备名}/thing/property/set
 * QoS: 1 (至少一次送达)
 */
void OnenetClient::SubscribeTopics() {
    if (!mqtt_client_ || !connected_) return;

    char topic[256];
    snprintf(topic, sizeof(topic),
        "$sys/%s/%s/thing/property/set",
        ONENET_PRODUCT_ID, ONENET_DEVICE_NAME);
    int msg_id = esp_mqtt_client_subscribe((esp_mqtt_client_handle_t)mqtt_client_, topic, 1);
    ESP_LOGI(TAG, "已订阅: %s (msg_id=%d)", topic, msg_id);
}

/**
 * 建立 MQTT TCP 连接
 * 使用非 TLS 连接 (MQTT_TRANSPORT_OVER_TCP), 端口 1883
 * 认证: username=产品ID, password=Token
 */
void OnenetClient::Connect() {
    if (mqtt_client_ || connected_) return;       // 已在连接中

    esp_mqtt_client_config_t mqtt_cfg = {};
    mqtt_cfg.broker.address.hostname = ONENET_MQTT_BROKER;
    mqtt_cfg.broker.address.port = ONENET_MQTT_PORT;
    mqtt_cfg.broker.address.transport = MQTT_TRANSPORT_OVER_TCP;        // 非 TLS
    mqtt_cfg.credentials.client_id = ONENET_DEVICE_NAME;
    mqtt_cfg.credentials.username = ONENET_PRODUCT_ID;
    mqtt_cfg.credentials.authentication.password = ONENET_PASSWORD;     // Token 认证

    ESP_LOGI(TAG, "正在连接OneNET MQTT: %s:%d", ONENET_MQTT_BROKER, ONENET_MQTT_PORT);

    mqtt_client_ = esp_mqtt_client_init(&mqtt_cfg);
    if (!mqtt_client_) {
        ESP_LOGE(TAG, "MQTT客户端创建失败");
        return;
    }

    // 注册全局事件处理器 (所有 MQTT 事件)
    esp_mqtt_client_register_event((esp_mqtt_client_handle_t)mqtt_client_,
        (esp_mqtt_event_id_t)ESP_EVENT_ANY_ID, MqttEventHandler, this);
    esp_mqtt_client_start((esp_mqtt_client_handle_t)mqtt_client_);
}

/**
 * 上传传感器数据到 OneNET 云平台
 * 发布到 thing/property/post, QoS 1
 *
 * JSON 格式 (OneNET 物模型):
 *   {"id":"<msg_id>","version":"1.0","params":{
 *     "temperature":{"value":xx.x},
 *     "humidity":{"value":xx.x},
 *     "light":{"value":xxx},
 *     "soil_moisture_percent":{"value":xx},
 *     "soil_moisture_raw":{"value":xxxxx},
 *     "relay_pump":{"value":0|1},
 *     "relay_light":{"value":0|1},
 *     "relay_heater":{"value":0|1}
 *   }}
 */
bool OnenetClient::UploadSensorData(const SensorData& data) {
    if (!mqtt_client_ || !connected_) return false;

    char topic[256];
    snprintf(topic, sizeof(topic),
        "$sys/%s/%s/thing/property/post",
        ONENET_PRODUCT_ID, ONENET_DEVICE_NAME);

    char payload[640];
    snprintf(payload, sizeof(payload),
        "{\"id\":\"%d\",\"version\":\"1.0\",\"params\":{"
        "\"temperature\":{\"value\":%.1f},"
        "\"humidity\":{\"value\":%.1f},"
        "\"light\":{\"value\":%d},"
        "\"soil_moisture_percent\":{\"value\":%d},"
        "\"soil_moisture_raw\":{\"value\":%d},"
        "\"relay_pump\":{\"value\":%d},"
        "\"relay_light\":{\"value\":%d},"
        "\"relay_heater\":{\"value\":%d}"
        "}}",
        msg_id_++,                                 // 消息 ID 递增
        data.temperature, data.humidity,
        data.light_value,
        data.soil_moisture_percent, data.soil_moisture_raw,
        data.relay_pump ? 1 : 0,
        data.relay_light ? 1 : 0,
        data.relay_heater ? 1 : 0);

    int ret = esp_mqtt_client_publish((esp_mqtt_client_handle_t)mqtt_client_,
        topic, payload, 0, 1, 0);                 // QoS 1, 不保留
    if (ret >= 0) {
        ESP_LOGI(TAG, "数据已上传: T=%.1f℃ H=%.1f%% L=%d S=%d%%",
                 data.temperature, data.humidity, data.light_value, data.soil_moisture_percent);
        return true;
    }
    ESP_LOGW(TAG, "数据上传失败");
    return false;
}

/**
 * 上报设备事件 (继电器状态变更)
 * 发布到 thing/event/post, QoS 0 (尽力送达)
 */
bool OnenetClient::ReportDeviceStatus(const SensorData& data) {
    if (!mqtt_client_ || !connected_) return false;

    char topic[256];
    snprintf(topic, sizeof(topic),
        "$sys/%s/%s/thing/event/post",
        ONENET_PRODUCT_ID, ONENET_DEVICE_NAME);

    char payload[384];
    snprintf(payload, sizeof(payload),
        "{\"id\":\"%d\",\"version\":\"1.0\",\"params\":{"
        "\"event_type\":\"relay_status\","
        "\"pump\":%d,\"light\":%d,\"heater\":%d"
        "}}",
        msg_id_++,
        data.relay_pump ? 1 : 0,
        data.relay_light ? 1 : 0,
        data.relay_heater ? 1 : 0);

    int ret = esp_mqtt_client_publish((esp_mqtt_client_handle_t)mqtt_client_,
        topic, payload, 0, 0, 0);                 // QoS 0
    return ret >= 0;
}

/**
 * 定时上传回调 — 15秒周期
 * 如果未连接且配置有效 → 尝试重连
 * 如果已连接 → 上传传感器数据
 */
void OnenetClient::UploadTimerCallback(void* arg) {
    auto* self = static_cast<OnenetClient*>(arg);
    if (!self->connected_ && self->config_ok_) {
        self->Connect();                           // 自动重连
        return;
    }
    if (!self->connected_) return;

    auto& monitor = GetPlantMonitor();
    self->UploadSensorData(monitor.GetSensorData());
}

/**
 * OneNET 客户端初始化
 * 检查配置有效性 → 创建 15 秒上传定时器
 * 连接在实际调用 Connect() 时建立 (由 UploadTimerCallback 延迟触发)
 */
bool OnenetClient::Initialize() {
    if (initialized_) {
        ESP_LOGW(TAG, "OneNET已经初始化");
        return true;
    }

    // 检查是否配置了有效的 OneNET Product ID
    if (strcmp(ONENET_PRODUCT_ID, "your_product_id") == 0) {
        ESP_LOGW(TAG, "OneNET未配置，跳过连接");
        config_ok_ = false;
        return false;
    }

    config_ok_ = true;
    ESP_LOGI(TAG, "OneNET客户端已就绪，等待WiFi连接后自动上线...");

    // 创建 15 秒周期上传定时器
    // 首次触发时: 如果 WiFi 已连接 → 建立 MQTT 连接
    // 后续触发: 上传传感器数据
    esp_timer_create_args_t timer_args = {
        .callback = UploadTimerCallback,
        .arg = this,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "onenet_upload_timer",
        .skip_unhandled_events = false,
    };
    ESP_ERROR_CHECK(esp_timer_create(&timer_args, &upload_timer_));
    ESP_ERROR_CHECK(esp_timer_start_periodic(upload_timer_, 15000000)); // 15,000,000 μs = 15s

    initialized_ = true;
    return true;
}
