/* MQTT地址与端口 */
#define HOST_RUL            "mqtt://mqtts.heclouds.com"                                        
#define HOST_PORT           1883                                                                
/* 根据三元组内容计算得出的数值 */
#define CLIENT_ID           "esp32s3"          /* 客户端ID */
#define USER_NAME           "Kp43RWJB3j"        /* 客户端用户名 */
#define PASSWORD            "version=2018-10-31&res=products%2FKp43RWJB3j%2Fdevices%2Fesp32s3&et=2524579200&method=md5&sign=V%2FQ7odRsYd8uU2ikvi3o8w%3D%3D                          /* 由MQTT_Password工具计算得出的连接密码 */
/* 发布与订阅 */
#define DEVICE_PUBLISH      "$sys/" USER_NAME "/" CLIENT_ID "/thing/property/post"      /* 发布主题 */
#define DEVICE_SUBSCRIBE    "$sys/" USER_NAME "/" CLIENT_ID "/thing/property/set"       /* 订阅主题 */
void lwip_demo(void);

/**
 * @brief       lwip_demo进程
 * @param       无
 * @retval      无
 */
void lwip_demo(void)
{
	uint8_t temperature = 0;  //真实项目可以从传感器中获取
    /* 设置MQTT客户端配置 */ 
    esp_mqtt_client_config_t mqtt_cfg = 
	{
        .broker.address.uri = HOST_RUL,
		.broker.address.port = HOST_PORT,
        .credentials.client_id = CLIENT_ID,
		.credentials.username = USER_NAME,
        .credentials.authentication.password = PASSWORD,
    };
 
    esp_mqtt_client_handle_t client = esp_mqtt_client_init(&mqtt_cfg);
	if (client == NULL) 
	{
        ESP_LOGE(TAG, "MQTT客户端初始化失败");
        return;
    }
    esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_err_t err = esp_mqtt_client_start(client);
	if (err != ESP_OK) 
	{
        ESP_LOGE(TAG, "MQTT客户端启动失败，错误码: %d", err);
        return;
    }
    
    ESP_LOGI(TAG, "MQTT客户端初始化完成");
 
    while(1)
    {
        if (g_publish_flag == 1)
        {
			temperature = ((temperature > 100) ? 0:(temperature + 10));
			sprintf(mqtt_publish_data, 
                   "{\"id\": \"123\",\"version\": \"1.0\",\"params\": {\"temperature\": {\"value\": %d}}}", 
                   temperature);
			/* 定期发布数据 */
            int msg_id = esp_mqtt_client_publish(client, DEVICE_PUBLISH, mqtt_publish_data, 0, 1, 0);
            if (msg_id < 0) 
			{
                ESP_LOGE(TAG, "MQTT发布失败");
            } 
			else 
			{
                ESP_LOGI(TAG, "MQTT发布成功，msg_id = %d", msg_id);
            }
        }
 
        vTaskDelay(pdMS_TO_TICKS(10000)); // 每10秒发布一次数据
    }
}

int g_publish_flag = 0;/* 发布成功标志位 */
static const char *TAG = "ONENET_MQTT";
char mqtt_publish_data[200] = {'0'};
 
static const char test_data[] = "{"
    "\"id\": \"123\","
    "\"version\": \"1.0\","
    "\"params\": {"
        "\"temperature\": {"
            "\"value\": 10"
        "}"
    "}"
"}";
 
/**
 * @brief       错误日记
 * @param       message     :错误消息
 * @param       error_code  :错误码
 * @retval      无
 */
static void log_error_if_nonzero(const char *message, int error_code)
{
    if (error_code != 0)
    {
        ESP_LOGE(TAG, "Last error %s: 0x%x", message, error_code);
    }
}
 
/**
 * @brief       注册接收MQTT事件的事件处理程序
 * @param       handler_args:注册到事件的用户数据
 * @param       base        :处理程序的事件库
 * @param       event_id    :接收到的事件的id
 * @param       event_data  :事件的数据
 * @retval      无
 */
static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    ESP_LOGD(TAG, "Event dispatched from event loop base=%s, event_id=%" PRIi32 "", base, event_id);
    esp_mqtt_event_handle_t event = event_data;
    esp_mqtt_client_handle_t client = event->client;
    int msg_id;
 
    switch ((esp_mqtt_event_id_t)event_id)
    {
        case MQTT_EVENT_CONNECTED:      /* 连接事件 */
            ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
            /* 订阅主题 */
            msg_id = esp_mqtt_client_subscribe(client, DEVICE_SUBSCRIBE, 1);
            ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);
            
            /* 发布测试数据 */
            msg_id = esp_mqtt_client_publish(client, DEVICE_PUBLISH, test_data, 0, 1, 0);
            ESP_LOGI(TAG, "sent publish successful, msg_id=%d", msg_id);
            
            g_publish_flag = 1;
            break;
            
        case MQTT_EVENT_DISCONNECTED:   /* 断开连接事件 */
            ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
            g_publish_flag = 0;
            break;
 
        case MQTT_EVENT_SUBSCRIBED:     /* 订阅成功事件 */
            ESP_LOGI(TAG, "MQTT_EVENT_SUBSCRIBED, msg_id=%d", event->msg_id);
            break;
            
        case MQTT_EVENT_UNSUBSCRIBED:   /* 取消订阅事件 */
            ESP_LOGI(TAG, "MQTT_EVENT_UNSUBSCRIBED, msg_id=%d", event->msg_id);
            break;
            
        case MQTT_EVENT_PUBLISHED:      /* 发布事件 */
            ESP_LOGI(TAG, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
            break;
            
        case MQTT_EVENT_DATA:           /* 接收数据事件 */
            ESP_LOGI(TAG, "MQTT_EVENT_DATA");
            printf("TOPIC=%.*s\r\n", event->topic_len, event->topic);
            printf("DATA=%.*s\r\n", event->data_len, event->data);
            
            /* 处理接收到的数据 */
            if (event->data_len > 0) {
                // 在这里添加处理接收数据的代码
            }
            break;
            
        case MQTT_EVENT_ERROR:
            ESP_LOGI(TAG, "MQTT_EVENT_ERROR");
            if (event->error_handle->error_type == MQTT_ERROR_TYPE_TCP_TRANSPORT)
            {
                log_error_if_nonzero("reported from esp-tls", event->error_handle->esp_tls_last_esp_err);
                log_error_if_nonzero("reported from tls stack", event->error_handle->esp_tls_stack_err);
                log_error_if_nonzero("captured as transport's socket errno",  event->error_handle->esp_transport_sock_errno);
                ESP_LOGI(TAG, "Last errno string (%s)", strerror(event->error_handle->esp_transport_sock_errno));
            }
            break;
            
        default:
            ESP_LOGI(TAG, "Other event id:%d", event->event_id);
            break;
    }
}