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
    msg_id_ = 0;
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

void OnenetClient::MqttEventHandler(void* handler_args, esp_event_base_t base,
                                    int32_t event_id, void* event_data) {
    auto* self = static_cast<OnenetClient*>(handler_args);
    auto* event = static_cast<esp_mqtt_event_handle_t>(event_data);

    switch ((esp_mqtt_event_id_t)event_id) {
        case MQTT_EVENT_CONNECTED:
            ESP_LOGI(TAG, "OneNET MQTT已连接");
            self->connected_ = true;
            self->SubscribeTopics();
            break;

        case MQTT_EVENT_DISCONNECTED:
            ESP_LOGW(TAG, "OneNET MQTT断开连接");
            self->connected_ = false;
            break;

        case MQTT_EVENT_DATA: {
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

void OnenetClient::HandleMqttData(const std::string& topic, const std::string& data) {
    if (!command_callback_) return;

    cJSON* root = cJSON_Parse(data.c_str());
    if (!root) return;

    cJSON* params = cJSON_GetObjectItem(root, "params");
    std::string cmd_json;
    if (params) {
        char* str = cJSON_PrintUnformatted(params);
        cmd_json = str;
        cJSON_free(str);
    } else {
        cmd_json = data;
    }

    std::string response = command_callback_(cmd_json);

    // 如果是cmd/request命令，回复响应
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

    // 如果是属性设置，回复 set_reply
    if (topic.find("thing/property/set") != std::string::npos) {
        cJSON* id_json = cJSON_GetObjectItem(root, "id");
        std::string msg_id = id_json && cJSON_IsString(id_json) ? id_json->valuestring : "0";
        char reply_topic[256];
        snprintf(reply_topic, sizeof(reply_topic),
            "$sys/%s/%s/thing/property/set_reply",
            ONENET_PRODUCT_ID, ONENET_DEVICE_NAME);
        char reply_payload[128];
        snprintf(reply_payload, sizeof(reply_payload),
            "{\"id\":\"%s\",\"code\":200,\"data\":{}}", msg_id.c_str());
        esp_mqtt_client_publish((esp_mqtt_client_handle_t)mqtt_client_,
            reply_topic, reply_payload, strlen(reply_payload), 0, 0);
    }

    cJSON_Delete(root);
}

void OnenetClient::SubscribeTopics() {
    if (!mqtt_client_ || !connected_) return;

    char topic[256];
    snprintf(topic, sizeof(topic),
        "$sys/%s/%s/thing/property/set",
        ONENET_PRODUCT_ID, ONENET_DEVICE_NAME);
    int msg_id = esp_mqtt_client_subscribe((esp_mqtt_client_handle_t)mqtt_client_, topic, 1);
    ESP_LOGI(TAG, "已订阅: %s (msg_id=%d)", topic, msg_id);
}

void OnenetClient::Connect() {
    if (mqtt_client_ || connected_) return;

    esp_mqtt_client_config_t mqtt_cfg = {};
    mqtt_cfg.broker.address.hostname = ONENET_MQTT_BROKER;
    mqtt_cfg.broker.address.port = ONENET_MQTT_PORT;
    mqtt_cfg.broker.address.transport = MQTT_TRANSPORT_OVER_TCP;
    mqtt_cfg.credentials.client_id = ONENET_DEVICE_NAME;
    mqtt_cfg.credentials.username = ONENET_PRODUCT_ID;
    mqtt_cfg.credentials.authentication.password = ONENET_PASSWORD;

    ESP_LOGI(TAG, "正在连接OneNET MQTT: %s:%d", ONENET_MQTT_BROKER, ONENET_MQTT_PORT);

    mqtt_client_ = esp_mqtt_client_init(&mqtt_cfg);
    if (!mqtt_client_) {
        ESP_LOGE(TAG, "MQTT客户端创建失败");
        return;
    }

    esp_mqtt_client_register_event((esp_mqtt_client_handle_t)mqtt_client_,
        (esp_mqtt_event_id_t)ESP_EVENT_ANY_ID, MqttEventHandler, this);
    esp_mqtt_client_start((esp_mqtt_client_handle_t)mqtt_client_);
}

bool OnenetClient::UploadSensorData(const SensorData& data) {
    if (!mqtt_client_ || !connected_) return false;

    char topic[256];
    snprintf(topic, sizeof(topic),
        "$sys/%s/%s/thing/property/post",
        ONENET_PRODUCT_ID, ONENET_DEVICE_NAME);

    char payload[512];
    snprintf(payload, sizeof(payload),
        "{\"id\":\"%d\",\"version\":\"1.0\",\"params\":{"
        "\"temperature\":{\"value\":%.1f},"
        "\"humidity\":{\"value\":%.1f},"
        "\"light\":{\"value\":%d},"
        "\"soil_moisture\":{\"value\":%d},"
        "\"relay_pump\":{\"value\":%d},"
        "\"relay_light\":{\"value\":%d},"
        "\"relay_heater\":{\"value\":%d}"
        "}}",
        msg_id_++, data.temperature, data.humidity,
        data.light_value, data.soil_moisture,
        data.relay_pump ? 1 : 0,
        data.relay_light ? 1 : 0,
        data.relay_heater ? 1 : 0);

    int ret = esp_mqtt_client_publish((esp_mqtt_client_handle_t)mqtt_client_,
        topic, payload, 0, 1, 0);
    if (ret >= 0) {
        ESP_LOGI(TAG, "数据已上传: T=%.1f℃ H=%.1f%% L=%d", data.temperature, data.humidity, data.light_value);
        return true;
    }
    ESP_LOGW(TAG, "数据上传失败");
    return false;
}

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
        msg_id_++, data.relay_pump ? 1 : 0,
        data.relay_light ? 1 : 0, data.relay_heater ? 1 : 0);

    int ret = esp_mqtt_client_publish((esp_mqtt_client_handle_t)mqtt_client_,
        topic, payload, 0, 0, 0);
    return ret >= 0;
}

void OnenetClient::UploadTimerCallback(void* arg) {
    auto* self = static_cast<OnenetClient*>(arg);
    if (!self->connected_ && self->config_ok_) {
        self->Connect();
        return;
    }
    if (!self->connected_) return;

    auto& monitor = GetPlantMonitor();
    self->UploadSensorData(monitor.GetSensorData());
}

bool OnenetClient::Initialize() {
    if (initialized_) {
        ESP_LOGW(TAG, "OneNET已经初始化");
        return true;
    }

    if (strcmp(ONENET_PRODUCT_ID, "your_product_id") == 0) {
        ESP_LOGW(TAG, "OneNET未配置，跳过连接");
        config_ok_ = false;
        return false;
    }

    config_ok_ = true;
    ESP_LOGI(TAG, "OneNET客户端已就绪，等待WiFi连接后自动上线...");

    esp_timer_create_args_t timer_args = {
        .callback = UploadTimerCallback,
        .arg = this,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "onenet_upload_timer",
        .skip_unhandled_events = false,
    };
    ESP_ERROR_CHECK(esp_timer_create(&timer_args, &upload_timer_));
    ESP_ERROR_CHECK(esp_timer_start_periodic(upload_timer_, 15000000));

    initialized_ = true;
    return true;
}
