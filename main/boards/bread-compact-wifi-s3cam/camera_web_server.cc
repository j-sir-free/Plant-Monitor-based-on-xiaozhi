/**
 * @file    camera_web_server.cc
 * @brief   摄像头 Web 服务器实现 — HTTP 端点 + MJPEG 流 + JSON API
 * @author  JJD-YOURFATHER
 *
 * 基于 ESP-IDF httpd 组件的轻量级 Web 服务器, 端口 80。
 * 在 WiFi 连接后由 DisplayUpdateCallback 懒启动。
 *
 * 4 个 HTTP 端点:
 *   GET /              — HTML5 仪表盘 (内嵌 JS, 2秒轮询)
 *   GET /stream        — MJPEG 视频流 (multipart/x-mixed-replace, ~15fps)
 *   GET /snapshot      — JPEG 单帧快照 (80% 质量)
 *   GET /api/sensors   — JSON 传感器数据 API (兼容旧版 soil_moisture 字段)
 *
 * 网页界面:
 *   2×2 网格卡片 (温度/湿度/光照/土壤) + 阈值显示 + 继电器状态徽章
 *   深色主题 (#1a1a2e), 移动端响应式 (max-width:480px)
 */
#include "camera_web_server.h"
#include "plant_monitor.h"
#include "board.h"
#include <esp_log.h>
#include <esp_http_server.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <cstring>
#include <cstdio>

static const char* TAG = "CameraWeb";

/**
 * 内嵌 HTML5 仪表盘页面 (C++11 原始字符串字面量 R"raw(...)raw")
 *
 * 前端架构:
 *   - 纯 HTML + CSS + Vanilla JS, 无外部依赖
 *   - fetch() API 每 2 秒轮询 /api/sensors
 *   - 继电器状态用 CSS class 切换 (r-on=绿色, r-off=灰色)
 *   - 向后兼容: 优先使用 soil_moisture_percent, 降级到旧版 soil_moisture 字段
 */
static const char HTML_PAGE[] = R"raw(
<!DOCTYPE html>
<html lang="zh-CN">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Plant Monitor</title>
<style>
*{margin:0;padding:0;box-sizing:border-box}
body{background:#1a1a2e;color:#eee;font-family:Arial,sans-serif;text-align:center}
h2{padding:12px 0 4px;font-size:18px;color:#aaa}
img{width:100%;max-width:480px;border:2px solid #333;border-radius:4px}
.grid{display:grid;grid-template-columns:1fr 1fr;gap:8px;padding:8px 12px;max-width:480px;margin:0 auto}
.card{background:#16213e;border-radius:6px;padding:8px}
.card .label{font-size:11px;color:#888}
.card .value{font-size:20px;font-weight:bold;margin-top:2px}
.t0{color:#00FFAA}.t1{color:#00AAFF}.t2{color:#FFFF88}.t3{color:#88FF88}
.relay{display:inline-block;padding:2px 10px;border-radius:10px;font-size:12px;margin:2px}
.r-on{background:#0a4;color:#fff}.r-off{background:#444;color:#999}
.threshold{font-size:11px;color:#888;margin-top:2px}
.footer{margin:8px 0 16px;font-size:11px;color:#555}
</style>
</head>
<body>
<h2>Plant Monitor</h2>
<img id="stream" src="/stream" alt="Camera Stream">
<div class="grid">
  <div class="card"><div class="label">Temperature</div><div class="value t0" id="temp">--</div><div class="threshold" id="temp_th">--</div></div>
  <div class="card"><div class="label">Humidity</div><div class="value t1" id="humi">--</div><div class="threshold" id="humi_th">--</div></div>
  <div class="card"><div class="label">Light</div><div class="value t2" id="light">--</div><div class="threshold" id="light_th">--</div></div>
  <div class="card"><div class="label">Soil</div><div class="value t3" id="soil">--</div><div class="threshold" id="soil_th">--</div></div>
</div>
<div>
  <span class="relay r-off" id="rpump">PUMP</span>
  <span class="relay r-off" id="rlight">LIGHT</span>
  <span class="relay r-off" id="rheat">HEAT</span>
</div>
<div class="footer">ESP32-S3-CAM Plant Monitor</div>
<script>
/** 每2秒轮询传感器数据并更新DOM */
function update(){
  fetch('/api/sensors').then(r=>r.json()).then(d=>{
    document.getElementById('temp').textContent=d.temperature.toFixed(1)+' C';
    document.getElementById('humi').textContent=d.humidity.toFixed(0)+' %';
    document.getElementById('light').textContent=d.light;
    // 向后兼容: 优先 soil_moisture_percent (新固件), 降级到 soil_moisture (旧固件)
    document.getElementById('soil').textContent=(d.soil_moisture_percent!==undefined?d.soil_moisture_percent+'%':(d.soil_moisture?'OK':'DRY'));
    document.getElementById('temp_th').textContent='T:'+d.thresholds.temp_min+'-'+d.thresholds.temp_max;
    document.getElementById('humi_th').textContent='H:'+d.thresholds.humidity_min+'-'+d.thresholds.humidity_max;
    document.getElementById('light_th').textContent='L:'+d.thresholds.light_min+'-'+d.thresholds.light_max;
    document.getElementById('soil_th').textContent='S:'+(d.thresholds.soil_moisture_dry!==undefined?d.thresholds.soil_moisture_dry+'%':'--');
    // 继电器状态: CSS class 切换
    var s=document.getElementById('rpump');s.textContent='PUMP';s.className='relay '+(d.relay_pump?'r-on':'r-off');
    s=document.getElementById('rlight');s.textContent='LIGHT';s.className='relay '+(d.relay_light?'r-on':'r-off');
    s=document.getElementById('rheat');s.textContent='HEAT';s.className='relay '+(d.relay_heater?'r-on':'r-off');
  }).catch(e=>console.log(e));  // 静默处理网络错误
}
setInterval(update,2000);   // 每 2000ms 轮询
update();                    // 立即执行首次更新
</script>
</body>
</html>
)raw";

CameraWebServer* CameraWebServer::instance_ = nullptr;

CameraWebServer::CameraWebServer() : server_handle_(nullptr) {
    instance_ = this;                            // 单例指针 (供静态 handler 访问 PlantMonitor)
}

CameraWebServer::~CameraWebServer() {
    Stop();
    instance_ = nullptr;
}

/**
 * 启动 HTTP 服务器 + 注册 4 个路由
 * 使用 LRU (Least Recently Used) 清理机制自动回收长时间不活跃的连接
 */
bool CameraWebServer::Start(int port) {
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = port;
    config.max_uri_handlers = 8;                // 最大 8 个并发处理
    config.lru_purge_enable = true;             // 自动清理僵尸连接

    if (httpd_start(&server_handle_, &config) != ESP_OK) {
        ESP_LOGE(TAG, "HTTP server start failed");
        return false;
    }

    // 注册 4 个 HTTP GET 路由
    httpd_uri_t uri_index = {
        .uri = "/", .method = HTTP_GET,
        .handler = IndexHandler, .user_ctx = nullptr,
    };
    httpd_register_uri_handler(server_handle_, &uri_index);

    httpd_uri_t uri_stream = {
        .uri = "/stream", .method = HTTP_GET,
        .handler = StreamHandler, .user_ctx = nullptr,
    };
    httpd_register_uri_handler(server_handle_, &uri_stream);

    httpd_uri_t uri_snapshot = {
        .uri = "/snapshot", .method = HTTP_GET,
        .handler = SnapshotHandler, .user_ctx = nullptr,
    };
    httpd_register_uri_handler(server_handle_, &uri_snapshot);

    httpd_uri_t uri_api = {
        .uri = "/api/sensors", .method = HTTP_GET,
        .handler = ApiSensorsHandler, .user_ctx = nullptr,
    };
    httpd_register_uri_handler(server_handle_, &uri_api);

    ESP_LOGI(TAG, "HTTP server started on port %d", port);
    return true;
}

void CameraWebServer::Stop() {
    if (server_handle_) {
        httpd_stop(server_handle_);
        server_handle_ = nullptr;
        ESP_LOGI(TAG, "HTTP server stopped");
    }
}

/** 首页 — 返回内嵌 HTML 页面, 禁用缓存 */
esp_err_t CameraWebServer::IndexHandler(httpd_req_t* req) {
    httpd_resp_set_type(req, "text/html; charset=utf-8");
    httpd_resp_set_hdr(req, "Cache-Control", "no-cache");
    httpd_resp_send(req, HTML_PAGE, strlen(HTML_PAGE));
    return ESP_OK;
}

/**
 * MJPEG 实时视频流
 * 算法: 循环捕获帧 → JPEG 编码 (70%质量) → 发送 multipart 块 → 延时 65ms (~15fps)
 * 客户端断开 (httpd_resp_send_chunk 失败) 或连续 10 次捕获失败时自动退出
 */
esp_err_t CameraWebServer::StreamHandler(httpd_req_t* req) {
    auto* camera = Board::GetInstance().GetCamera();
    if (camera == nullptr) {
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    // MJPEG 内容类型: multipart/x-mixed-replace, 每个 part 以 --frame 分隔
    httpd_resp_set_type(req, "multipart/x-mixed-replace; boundary=frame");
    httpd_resp_set_hdr(req, "Cache-Control", "no-cache");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*"); // CORS

    char buf[256];
    std::string jpeg_data;
    int fail_count = 0;

    while (true) {
        if (!camera->Capture()) {               // 抓取一帧
            fail_count++;
            if (fail_count > 10) break;          // 连续失败 >10 → 退出
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }
        fail_count = 0;

        jpeg_data.clear();
        if (!camera->EncodeCurrentFrameToJpeg(jpeg_data, 70)) { // JPEG 质量 70
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        // MJPEG multipart 帧格式:
        //   --frame\r\n
        //   Content-Type: image/jpeg\r\n
        //   Content-Length: <size>\r\n
        //   \r\n
        //   <JPEG data>\r\n
        int header_len = snprintf(buf, sizeof(buf),
            "--frame\r\nContent-Type: image/jpeg\r\nContent-Length: %d\r\n\r\n",
            (int)jpeg_data.size());

        if (httpd_resp_send_chunk(req, buf, header_len) != ESP_OK) break;
        if (httpd_resp_send_chunk(req, jpeg_data.c_str(), jpeg_data.size()) != ESP_OK) break;
        if (httpd_resp_send_chunk(req, "\r\n", 2) != ESP_OK) break;

        vTaskDelay(pdMS_TO_TICKS(65));           // ~15fps (1000/65≈15)
    }

    ESP_LOGI(TAG, "MJPEG client disconnected");
    return ESP_OK;
}

/** 单帧 JPEG 快照 — 捕获一帧, JPEG 编码 (80%质量), 返回 image/jpeg */
esp_err_t CameraWebServer::SnapshotHandler(httpd_req_t* req) {
    auto* camera = Board::GetInstance().GetCamera();
    if (camera == nullptr) {
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    if (!camera->Capture()) {
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    std::string jpeg_data;
    if (!camera->EncodeCurrentFrameToJpeg(jpeg_data, 80)) { // 80% 质量
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    httpd_resp_set_type(req, "image/jpeg");
    httpd_resp_set_hdr(req, "Cache-Control", "no-cache");
    httpd_resp_send(req, jpeg_data.c_str(), jpeg_data.size());
    return ESP_OK;
}

/**
 * 传感器 JSON API — 返回当前传感器数据和阈值
 * JSON 结构:
 *   { temperature, humidity, light,
 *     soil_moisture_percent, soil_moisture_raw, soil_moisture_digital,
 *     relay_pump, relay_light, relay_heater,
 *     thresholds: { temp_min, temp_max, humidity_min, humidity_max,
 *                   light_min, light_max, soil_moisture_dry } }
 */
esp_err_t CameraWebServer::ApiSensorsHandler(httpd_req_t* req) {
    auto& monitor = GetPlantMonitor();
    auto data = monitor.GetSensorData();
    auto thresholds = monitor.GetThresholds();

    char buf[512];
    snprintf(buf, sizeof(buf),
        "{\"temperature\":%.1f,\"humidity\":%.1f,\"light\":%d,"
        "\"soil_moisture_percent\":%d,\"soil_moisture_raw\":%d,"
        "\"soil_moisture_digital\":%s,"
        "\"relay_pump\":%s,\"relay_light\":%s,"
        "\"relay_heater\":%s,\"thresholds\":{"
        "\"temp_min\":%d,\"temp_max\":%d,"
        "\"humidity_min\":%d,\"humidity_max\":%d,"
        "\"light_min\":%d,\"light_max\":%d,"
        "\"soil_moisture_dry\":%d}}",
        data.temperature, data.humidity, data.light_value,
        data.soil_moisture_percent, data.soil_moisture_raw,
        data.soil_moisture_digital ? "true" : "false",
        data.relay_pump ? "true" : "false",
        data.relay_light ? "true" : "false",
        data.relay_heater ? "true" : "false",
        thresholds.temp_min, thresholds.temp_max,
        thresholds.humidity_min, thresholds.humidity_max,
        thresholds.light_min, thresholds.light_max,
        thresholds.soil_moisture_dry);

    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-cache");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");  // CORS
    httpd_resp_send(req, buf, strlen(buf));
    return ESP_OK;
}
